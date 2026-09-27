<script setup lang="ts">
import { computed, nextTick, onBeforeUnmount, onMounted, ref } from 'vue'
import * as echarts from 'echarts'
import { HubConnectionBuilder, HubConnectionState } from '@microsoft/signalr'

type Frame = {
  seq: number; stamp_ns: string; run_id: string; valid: boolean; mode: string
  s_m: number | null; v_mps: number | null; control_u: number | null
  wheel_speeds_mps: Record<string, number>; uncertainty_available: boolean
  sigma_v_mps: number | null; reason_codes: string[]
}
type Run = { run_id: string; status?: string; model_version?: string; synthetic?: boolean }
type XY = { stamp_ns: string; x_m: number; y_m: number }
type Report = {
  metrics: Record<string, number | null | Record<string, number>>; unavailable_reason: string | null
  truth: { stamp_ns: string; v_mps: number; s_m: number }[]
  incidents: {stamp_ns: string; description: string}[]
  fault_windows?: { type: string; start_ns: string; end_ns: string; position_drift_m: number
    max_speed_error_mps: number; affected_channels?: string[] }[]
  scenario?: { name: string; fault: string; channels: string[]; bag: string; vehicle_id: string
    preset: string; window_s: number }
  reference?: { source: string; coverage: number }
  map?: { frame: string; route_id: string | null; routes: { route_id: string; xy: number[][] }[]
    reference: XY[]; estimate: XY[] }
  corrections?: { available: boolean; reason?: string; jump_m?: number | null
    accepted: { kind: string; count: number }[]; rejected: { kind: string; count: number }[] }
}
const runs = ref<Run[]>([]), selected = ref(''), frames = ref<Frame[]>([])
const report = ref<Report | null>(null), error = ref(''), connection = ref('Подключение…')
const lastReceived = ref(0), now = ref(Date.now())
const speedEl = ref<HTMLElement>(), pathEl = ref<HTMLElement>(), controlEl = ref<HTMLElement>()
const mapEl = ref<HTMLElement>(), errorEl = ref<HTMLElement>(), modeEl = ref<HTMLElement>()
let charts: echarts.ECharts[] = [], timer: ReturnType<typeof setInterval> | undefined
let renderTimer: ReturnType<typeof setTimeout> | undefined
let generation = 0
const hub = new HubConnectionBuilder().withUrl('/hubs/telemetry').withAutomaticReconnect().build()
const current = computed(() => runs.value.find(r => r.run_id === selected.value))
const last = computed(() => frames.value.at(-1))
const stale = computed(() => current.value?.status === 'running' && now.value - lastReceived.value > 2500)
const metricNames: Record<string,string> = {
  speed_rmse_mps: 'RMSE скорости, м/с', speed_p95_abs_mps: 'Скорость, 95-й процентиль, м/с',
  speed_bias_mps: 'Смещение скорости, м/с', position_rmse_m: 'RMSE пути, м',
  final_position_error_m: 'Ошибка пути в конце, м', valid_fraction: 'Доля валидного времени',
  position_p95_abs_m: 'Путь, 95-й процентиль, м', velocity_95pct_coverage: 'Покрытие интервалом 95%'
}
const modeOrder = ['INITIALIZING', 'FUSED', 'DEGRADED', 'MODEL_ONLY', 'INVALID']
const faultTitle: Record<string, string> = {
  freeze: 'заморозка', dropout: 'пропажа', spike: 'всплески', slip: 'проскальзывание', lock: 'блокировка'
}
const metricValue = (key: string) => {
  const value = report.value?.metrics[key]
  return typeof value === 'number' ? value.toFixed(3) : '—'
}
const faults = computed(() => report.value?.fault_windows ?? [])
const correctionCount = (items: { count: number }[] | undefined) =>
  (items ?? []).reduce((total, item) => total + item.count, 0)
async function json(url: string) {
  const response = await fetch(url)
  if (!response.ok) throw new Error('HTTP ' + response.status + ': ' + url)
  return response.json()
}
function merge(batch: Frame[]) {
  const map = new Map(frames.value.map(f => [f.seq, f]))
  for (const frame of batch) if (frame.run_id === selected.value) map.set(frame.seq, frame)
  frames.value = [...map.values()].sort((a,b) => a.seq-b.seq).slice(-20000)
  scheduleDraw()
}
async function refreshRuns() {
  try { runs.value = await json('/api/runs'); error.value = '' }
  catch (e) { error.value = String(e) }
}
async function loadRun(id: string) {
  const ticket = ++generation
  if (selected.value && hub.state === HubConnectionState.Connected)
    await hub.invoke('Unsubscribe', selected.value)
  selected.value = id; frames.value = []; report.value = null; error.value = ''
  try {
    if (hub.state === HubConnectionState.Connected) await hub.invoke('Subscribe', id)
    let after = -1
    while (true) {
      const batch: Frame[] = await json('/api/runs/' + id + '/telemetry?afterSeq=' + after + '&limit=5000')
      if (ticket !== generation) return
      merge(batch)
      if (batch.length < 5000) break
      after = batch.at(-1)!.seq
    }
    const response = await fetch('/api/runs/' + id + '/report')
    if (ticket !== generation) return
    report.value = response.ok ? await response.json() : null
    if (!response.ok && response.status !== 404) throw new Error('Report HTTP ' + response.status)
    scheduleDraw()
  } catch (e) { error.value = String(e) }
}
function scheduleDraw() {
  if (renderTimer) return
  renderTimer = setTimeout(() => { renderTimer = undefined; draw() }, 100)
}
function draw() {
  if (!charts.length) return
  const origin = BigInt(frames.value[0]?.stamp_ns ?? '0')
  const seconds = (stamp: string) => Number(BigInt(stamp)-origin)/1e9
  const series = (name: string, data: (number | null)[][], color?: string) =>
    ({ name, type: 'line' as const, showSymbol: false, connectNulls: false, data,
       lineStyle: { width: 2 }, ...(color ? { color } : {}) })
  const points = (get: (f: Frame) => number | null) => frames.value.map(f => [seconds(f.stamp_ns), get(f)])
  const common = { animation: false, tooltip: { trigger: 'axis' }, legend: { top: 6, textStyle: { color: '#9aaec3' } },
    grid: { left: 65, right: 25, top: 45, bottom: 40 },
    xAxis: { type: 'value', name: 'с', axisLabel: { color: '#9aaec3' } },
    yAxis: { type: 'value', axisLabel: { color: '#9aaec3' }, splitLine: { lineStyle: { color: '#203145' } } },
    dataZoom: [{ type: 'inside' }] }
  const channels = [...new Set(frames.value.flatMap(f => Object.keys(f.wheel_speeds_mps || {})))]
  const faultArea = faults.value.map(w => [
    { xAxis: seconds(w.start_ns), itemStyle: { color: 'rgba(255,162,142,0.12)' } }, { xAxis: seconds(w.end_ns) }])
  const withFaults = <T extends object>(s: T) => faultArea.length
    ? { ...s, markArea: { silent: true, data: faultArea } } : s
  const velocitySeries = [
    withFaults(series('Оценка', points(f => f.v_mps), '#43dfb4')),
    ...channels.map(id => series('Колесо ' + id, points(f => f.wheel_speeds_mps?.[id] ?? null))),
  ]
  if (report.value?.truth?.length)
    velocitySeries.push(series('Эталон', report.value.truth.map(t => [seconds(t.stamp_ns), t.v_mps]), '#f2be65'))
  if (frames.value.some(f => f.uncertainty_available))
    for (const sign of [-1,1])
      velocitySeries.push(series(sign < 0 ? 'Нижняя 95%' : 'Верхняя 95%',
        points(f => f.uncertainty_available && f.sigma_v_mps != null && f.v_mps != null
          ? f.v_mps + sign*1.96*f.sigma_v_mps : null), '#618e91'))
  charts[0].setOption({ ...common, series: velocitySeries }, true)
  const paths = [series('Оценка пути', points(f => f.s_m), '#43dfb4')]
  if (report.value?.truth?.length)
    paths.push(series('Эталон пути', report.value.truth.map(t => [seconds(t.stamp_ns), t.s_m]), '#f2be65'))
  charts[1].setOption({ ...common, series: paths }, true)
  charts[2].setOption({ ...common, series: [series('Ручка', points(f => f.control_u), '#a69fff')] }, true)
  drawMap(common)
  drawError(common, seconds, series, withFaults)
  charts[5].setOption({ ...common, legend: { show: false }, grid: { left: 110, right: 25, top: 20, bottom: 40 },
    yAxis: { type: 'category', data: modeOrder, axisLabel: { color: '#9aaec3' } },
    series: [{ name: 'Режим', type: 'line', step: 'end', showSymbol: false, color: '#43dfb4',
      data: frames.value.map(f => [seconds(f.stamp_ns), f.mode]) }] }, true)
}
function drawError(common: object, seconds: (s: string) => number, series: Function, withFaults: Function) {
  const truth = report.value?.truth ?? []
  const times = truth.map(t => seconds(t.stamp_ns))
  const nearest = (t: number) => {
    let lo = 0, hi = times.length - 1
    while (lo < hi) { const mid = (lo + hi) >> 1; if (times[mid] < t) lo = mid + 1; else hi = mid }
    return lo
  }
  const data = times.length ? frames.value.map(f => {
    const t = seconds(f.stamp_ns), i = nearest(t)
    if (f.v_mps == null || t < times[0] || t > times[times.length - 1] || Math.abs(times[i] - t) > 0.5)
      return [t, null]
    return [t, f.v_mps - truth[i].v_mps]
  }) : []
  charts[4].setOption({ ...common, series: [withFaults(series('Оценка − эталон', data, '#ffa28e'))] }, true)
}
function drawMap(common: object) {
  const map = report.value?.map
  const el = mapEl.value
  if (!map || !el) { charts[3].clear(); return }
  const track = [...map.reference, ...map.estimate]
  if (!track.length) { charts[3].clear(); return }
  const xs = track.map(p => p.x_m), ys = track.map(p => p.y_m)
  let [x0, x1, y0, y1] = [Math.min(...xs), Math.max(...xs), Math.min(...ys), Math.max(...ys)]
  const pad = Math.max(60, (x1 - x0 + y1 - y0) * 0.1)
  x0 -= pad; x1 += pad; y0 -= pad; y1 += pad
  // equal scale on both axes so that distances on the map are not distorted
  const width = Math.max(200, el.clientWidth - 100), height = Math.max(200, el.clientHeight - 70)
  const scale = Math.max((x1 - x0) / width, (y1 - y0) / height)
  const cx = (x0 + x1) / 2, cy = (y0 + y1) / 2
  const line = (name: string, data: number[][], color: string, lw = 2) =>
    ({ name, type: 'line' as const, showSymbol: false, data, color, lineStyle: { width: lw, color } })
  const xy = (rows: XY[]) => rows.map(p => [p.x_m, p.y_m])
  const marks = (rows: XY[], name: string, color: string) => rows.length ? [{
    name, type: 'scatter' as const, symbolSize: 9, color,
    data: [[rows[0].x_m, rows[0].y_m], [rows[rows.length - 1].x_m, rows[rows.length - 1].y_m]] }] : []
  charts[3].setOption({ ...common,
    legend: { top: 6, textStyle: { color: '#9aaec3' }, data: ['Pathgraph', 'Эталон GNSS', 'Оценка'] },
    tooltip: { trigger: 'item' },
    xAxis: { type: 'value', name: 'x, м', min: cx - scale * width / 2, max: cx + scale * width / 2,
      axisLabel: { color: '#9aaec3', formatter: (v: number) => String(Math.round(v)) }, splitLine: { show: false } },
    yAxis: { type: 'value', name: 'y, м', min: cy - scale * height / 2, max: cy + scale * height / 2,
      axisLabel: { color: '#9aaec3', formatter: (v: number) => String(Math.round(v)) },
      splitLine: { lineStyle: { color: '#203145' } } },
    grid: { left: 70, right: 30, top: 45, bottom: 40 },
    series: [
      ...map.routes.map((r, i) => ({ ...line(i ? 'Pathgraph ' + r.route_id : 'Pathgraph', r.xy, '#4a5d73', 2) })),
      line('Эталон GNSS', xy(map.reference), '#f2be65', 3),
      line('Оценка', xy(map.estimate), '#43dfb4', 2),
      ...marks(map.reference, 'Начало / конец эталона', '#f2be65'),
    ] }, true)
}
const resize = () => charts.forEach(c => c.resize())
hub.on('telemetryBatch', (batch: Frame[]) => { lastReceived.value = Date.now(); merge(batch) })
hub.onreconnecting(() => { connection.value = 'Связь потеряна, переподключение…' })
hub.onclose(() => { connection.value = 'Нет связи с API' })
hub.onreconnected(async () => {
  connection.value = 'Подключено'
  if (selected.value) await loadRun(selected.value)
})
onMounted(async () => {
  await nextTick()
  charts = [speedEl.value!, pathEl.value!, controlEl.value!, mapEl.value!, errorEl.value!, modeEl.value!]
    .map(el => echarts.init(el))
  window.addEventListener('resize', resize)
  try { await hub.start(); connection.value = 'Подключено' } catch { connection.value = 'Нет live-связи; доступны записи' }
  await refreshRuns()
  if (runs.value.length) await loadRun(runs.value[0].run_id)
  timer = setInterval(() => { now.value = Date.now(); void refreshRuns() }, 3000)
})
onBeforeUnmount(() => {
  if (timer) clearInterval(timer)
  if (renderTimer) clearTimeout(renderTimer)
  window.removeEventListener('resize', resize); charts.forEach(c => c.dispose()); void hub.stop()
})
</script>

<template>
  <div class="layout">
    <aside>
      <div class="brand">O<span>·</span>DOMETRY</div>
      <p class="muted">FAILURE LAB / 0.3</p>
      <h2>Эксперименты</h2>
      <button v-for="run in runs" :key="run.run_id" :class="{active: selected === run.run_id}"
        @click="loadRun(run.run_id)">
        {{ run.run_id }}<small>{{ run.status ?? '—' }}</small>
      </button>
      <p v-if="!runs.length" class="muted">Запустите CLI demo и импортируйте отчет.</p>
      <div class="connection">{{ connection }}</div>
    </aside>
    <main>
      <header><div><p class="eyebrow">РЕЗЕРВНАЯ ОДОМЕТРИЯ</p><h1>{{ selected || 'Ожидание эксперимента' }}</h1></div>
        <span class="badge">{{ current?.model_version ?? 'wheel-hold-v1' }}</span></header>
      <div v-if="error" class="alert">{{ error }}</div>
      <div v-if="stale" class="alert">Нет свежей телеметрии. Последние значения не являются текущими.</div>
      <section class="notice">Исследовательский baseline · неопределенность не калибрована
        <span v-if="current?.synthetic"> · синтетический сценарий</span>
      </section>
      <section v-if="report?.scenario" class="scenario">
        <span><small>Запись</small>{{ report.scenario.bag }}</span>
        <span><small>Трамвай</small>{{ report.scenario.vehicle_id }}</span>
        <span><small>Пресет</small>{{ report.scenario.preset }}</span>
        <span><small>Отказ</small>{{ report.scenario.fault === 'none' ? 'нет'
          : (faultTitle[report.scenario.fault] ?? report.scenario.fault) + ' · ' + report.scenario.channels.join(' + ') }}</span>
        <span><small>Эталон</small>GNSS, покрытие {{ ((report.reference?.coverage ?? 0) * 100).toFixed(0) }}%</span>
      </section>
      <div class="metrics">
        <article v-for="(label, key) in metricNames" :key="key"><small>{{ label }}</small>
          <strong>{{ metricValue(String(key)) }}</strong></article>
      </div>
      <section class="status"><span :class="['badge', last?.valid ? 'good' : 'bad']">{{ last?.mode ?? 'INITIALIZING' }}</span>
        <span>{{ last?.reason_codes?.join(' · ') ?? 'Нет данных' }}</span></section>
      <p v-if="report?.unavailable_reason" class="muted">Фактическая ошибка недоступна: {{ report.unavailable_reason }}</p>
      <section class="panel"><h2>Карта <small>Pathgraph, эталон GNSS и оценка на карте</small></h2>
        <div ref="mapEl" class="chart map"></div>
        <p class="muted small">{{ report?.map?.frame ?? 'Карта появится для записей с эталоном на Pathgraph.' }}</p></section>
      <section class="panel"><h2>Скорость <small>м/с; розовым отмечено окно отказа</small></h2>
        <div ref="speedEl" class="chart"></div></section>
      <section class="panel"><h2>Ошибка скорости <small>оценка − эталон, м/с</small></h2>
        <div ref="errorEl" class="chart short"></div></section>
      <section class="panel"><h2>Режим фильтра</h2><div ref="modeEl" class="chart short"></div></section>
      <section class="panel"><h2>Продольный путь <small>м</small></h2><div ref="pathEl" class="chart"></div></section>
      <section class="panel"><h2>Управление</h2><div ref="controlEl" class="chart short"></div></section>
      <section class="panel"><h2>Внесённые отказы</h2>
        <p v-for="(w, index) in faults" :key="index" class="incident">
          <strong>{{ faultTitle[w.type] ?? w.type }}</strong> · {{ (w.affected_channels ?? []).join(' + ') || 'колёса' }}
          — дрейф пути за окно {{ w.position_drift_m.toFixed(2) }} м,
          максимальная ошибка скорости {{ w.max_speed_error_mps.toFixed(2) }} м/с</p>
        <p v-if="!faults.length" class="muted">Отказы не вносились.</p>
      </section>
      <section class="panel"><h2>GNSS-коррекции</h2>
        <p v-if="report?.corrections && !report.corrections.available" class="muted">
          Недоступны: {{ report.corrections.reason }}. Принятые и отклонённые коррекции появятся здесь
          после подключения ядра.</p>
        <p v-else-if="report?.corrections">Принято: {{ correctionCount(report.corrections.accepted) }},
          отклонено: {{ correctionCount(report.corrections.rejected) }},
          максимальный скачок: {{ report.corrections.jump_m?.toFixed(3) ?? '—' }} м</p>
        <p v-else class="muted">Нет данных о коррекциях для этого запуска.</p>
      </section>
      <section class="panel"><h2>События качества</h2>
        <p v-for="(incident, index) in report?.incidents ?? []" :key="index" class="incident">
          <code>{{ incident.stamp_ns }} ns</code> {{ incident.description }}</p>
        <p v-if="!report" class="muted">Отчет появится после evaluate и import-report.</p>
      </section>
      <footer>Показаны последние {{ frames.length }} отсчетов, максимум 20 000. Полные записи используются в Failure Lab.</footer>
    </main>
  </div>
</template>
