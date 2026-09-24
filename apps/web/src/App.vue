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
type Report = {
  metrics: Record<string, number | null>; unavailable_reason: string | null
  truth: { stamp_ns: string; v_mps: number; s_m: number }[]
  incidents: {stamp_ns: string; description: string}[]
}
const runs = ref<Run[]>([]), selected = ref(''), frames = ref<Frame[]>([])
const report = ref<Report | null>(null), error = ref(''), connection = ref('Подключение…')
const lastReceived = ref(0), now = ref(Date.now())
const speedEl = ref<HTMLElement>(), pathEl = ref<HTMLElement>(), controlEl = ref<HTMLElement>()
let charts: echarts.ECharts[] = [], timer: ReturnType<typeof setInterval> | undefined
let renderTimer: ReturnType<typeof setTimeout> | undefined
let generation = 0
const hub = new HubConnectionBuilder().withUrl('/hubs/telemetry').withAutomaticReconnect().build()
const current = computed(() => runs.value.find(r => r.run_id === selected.value))
const last = computed(() => frames.value.at(-1))
const stale = computed(() => current.value?.status === 'running' && now.value - lastReceived.value > 2500)
const metricNames: Record<string,string> = {
  speed_rmse_mps: 'RMSE скорости, м/с', position_mae_m: 'MAE пути, м',
  final_position_error_m: 'Ошибка пути в конце, м', valid_fraction: 'Доля валидного времени'
}
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
  const common = { animation: false, tooltip: { trigger: 'axis' }, legend: { textStyle: { color: '#9aaec3' } },
    grid: { left: 65, right: 25, top: 45, bottom: 40 },
    xAxis: { type: 'value', name: 'с', axisLabel: { color: '#9aaec3' } },
    yAxis: { type: 'value', axisLabel: { color: '#9aaec3' }, splitLine: { lineStyle: { color: '#203145' } } },
    dataZoom: [{ type: 'inside' }] }
  const channels = [...new Set(frames.value.flatMap(f => Object.keys(f.wheel_speeds_mps || {})))]
  const velocitySeries = [
    series('Оценка', points(f => f.v_mps), '#43dfb4'),
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
  charts = [speedEl.value!, pathEl.value!, controlEl.value!].map(el => echarts.init(el))
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
      <p class="muted">FAILURE LAB / 0.2</p>
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
      <div class="metrics">
        <article v-for="(label, key) in metricNames" :key="key"><small>{{ label }}</small>
          <strong>{{ report?.metrics[key] == null ? '—' : Number(report.metrics[key]).toFixed(3) }}</strong></article>
      </div>
      <section class="status"><span :class="['badge', last?.valid ? 'good' : 'bad']">{{ last?.mode ?? 'INITIALIZING' }}</span>
        <span>{{ last?.reason_codes?.join(' · ') ?? 'Нет данных' }}</span></section>
      <p v-if="report?.unavailable_reason" class="muted">Фактическая ошибка недоступна: {{ report.unavailable_reason }}</p>
      <section class="panel"><h2>Скорость <small>м/с</small></h2><div ref="speedEl" class="chart"></div></section>
      <section class="panel"><h2>Продольный путь <small>м</small></h2><div ref="pathEl" class="chart"></div></section>
      <section class="panel"><h2>Управление</h2><div ref="controlEl" class="chart short"></div></section>
      <section class="panel"><h2>События качества</h2>
        <p v-for="(incident, index) in report?.incidents ?? []" :key="index" class="incident">
          <code>{{ incident.stamp_ns }} ns</code> {{ incident.description }}</p>
        <p v-if="!report" class="muted">Отчет появится после evaluate и import-report.</p>
      </section>
      <footer>Показаны последние {{ frames.length }} отсчетов, максимум 20 000. Полные записи используются в Failure Lab.</footer>
    </main>
  </div>
</template>
