using System.Text.Json;
using System.Text.RegularExpressions;
using Microsoft.AspNetCore.SignalR;
using Microsoft.Data.Sqlite;

var builder = WebApplication.CreateBuilder(args);
builder.WebHost.ConfigureKestrel(o => o.Limits.MaxRequestBodySize = 8 * 1024 * 1024);
builder.Services.AddSignalR(o => o.MaximumReceiveMessageSize = 32 * 1024);
builder.Services.AddCors(o => o.AddDefaultPolicy(p => p
    .WithOrigins("http://localhost:5173", "http://127.0.0.1:5173")
    .AllowAnyHeader().AllowAnyMethod().AllowCredentials()));
builder.Services.AddSingleton(new RunStore(
    Environment.GetEnvironmentVariable("ODOMETRY_DATA_DIR") ?? Path.Combine(AppContext.BaseDirectory, "data")));
var app = builder.Build();
app.Use(async (context, next) =>
{
    try { await next(); }
    catch (Exception ex) when (ex is ArgumentException or JsonException or FormatException or KeyNotFoundException
                               or InvalidOperationException or OverflowException)
    {
        context.Response.StatusCode = 400;
        await context.Response.WriteAsJsonAsync(new { error = ex.Message });
    }
});
app.UseCors();
app.UseDefaultFiles();
app.UseStaticFiles();
app.MapGet("/api/health", () => Results.Ok(new { status = "ready", schema_version = "0.2" }));
app.MapPost("/api/runs", (JsonElement body, RunStore store) =>
{
    var id = Validation.RunId(body.GetProperty("run_id").GetString());
    if (body.GetRawText().Length > 64_000) return Results.BadRequest(new { error = "Run metadata too large" });
    store.Register(id, body.GetRawText());
    return Results.Ok(new { run_id = id });
});
app.MapGet("/api/runs", (RunStore store) => Results.Json(store.List()));
app.MapGet("/api/runs/{id}", (string id, RunStore store) =>
{
    var json = store.Get(Validation.RunId(id));
    return json is null ? Results.NotFound() : Results.Content(json, "application/json");
});
app.MapPost("/api/runs/{id}/telemetry", async (string id, JsonElement body, RunStore store,
    IHubContext<TelemetryHub> hub) =>
{
    Validation.RunId(id);
    if (store.Get(id) is null) return Results.NotFound();
    if (body.ValueKind != JsonValueKind.Array || body.GetArrayLength() > 500)
        return Results.BadRequest(new { error = "Expected array with at most 500 frames" });
    var frames = body.EnumerateArray().Select(e => e.Clone()).ToArray();
    foreach (var frame in frames) Validation.Frame(id, frame);
    var accepted = store.Append(id, frames);
    if (accepted.Count > 0)
        await hub.Clients.Group(id).SendAsync("telemetryBatch", accepted);
    return Results.Ok(new { accepted = accepted.Count, duplicates = frames.Length - accepted.Count });
});
app.MapGet("/api/runs/{id}/telemetry", (string id, long? afterSeq, int? limit, RunStore store) =>
{
    Validation.RunId(id);
    if (store.Get(id) is null) return Results.NotFound();
    return Results.Json(store.Frames(id, afterSeq ?? -1, Math.Clamp(limit ?? 2000, 1, 5000)));
});
app.MapPut("/api/runs/{id}/report", (string id, JsonElement body, RunStore store) =>
{
    Validation.RunId(id);
    if (store.Get(id) is null) return Results.NotFound();
    if (body.ValueKind != JsonValueKind.Object || !body.TryGetProperty("metrics", out _))
        return Results.BadRequest(new { error = "Report must contain metrics" });
    store.SaveReport(id, body.GetRawText());
    return Results.Ok(new { stored = true });
});
app.MapGet("/api/runs/{id}/report", (string id, RunStore store) =>
{
    var report = store.Report(Validation.RunId(id));
    return report is null ? Results.NotFound() : Results.Content(report, "application/json");
});
app.MapHub<TelemetryHub>("/hubs/telemetry");
app.MapFallback(async context =>
{
    if (context.Request.Path.StartsWithSegments("/api") || context.Request.Path.StartsWithSegments("/hubs"))
    {
        context.Response.StatusCode = 404;
        return;
    }
    var index = Path.Combine(app.Environment.WebRootPath ?? "wwwroot", "index.html");
    if (!File.Exists(index)) { context.Response.StatusCode = 404; return; }
    context.Response.ContentType = "text/html";
    await context.Response.SendFileAsync(index);
});
app.Run();

public class TelemetryHub : Hub
{
    public Task Subscribe(string runId) => Groups.AddToGroupAsync(Context.ConnectionId, Validation.RunId(runId));
    public Task Unsubscribe(string runId) => Groups.RemoveFromGroupAsync(Context.ConnectionId, Validation.RunId(runId));
}

static class Validation
{
    public static string RunId(string? id)
    {
        if (id is null || !Regex.IsMatch(id, "^[A-Za-z0-9][A-Za-z0-9_-]{0,79}$"))
            throw new ArgumentException("Invalid run_id");
        return id;
    }
    public static void Frame(string id, JsonElement frame)
    {
        if (frame.GetRawText().Length > 32_000) throw new ArgumentException("Frame too large");
        if (frame.GetProperty("run_id").GetString() != id) throw new ArgumentException("run_id mismatch");
        var seq = frame.GetProperty("seq").GetInt64();
        if (seq < 0 || seq > 9_007_199_254_740_991) throw new ArgumentException("Invalid seq");
        var stamp = frame.GetProperty("stamp_ns").GetString();
        if (!long.TryParse(stamp, out var time) || time < 0) throw new ArgumentException("Invalid stamp_ns");
        if (frame.GetProperty("valid").ValueKind is not (JsonValueKind.True or JsonValueKind.False))
            throw new ArgumentException("valid must be boolean");
        if (frame.GetProperty("mode").GetString() is not
            ("INITIALIZING" or "FUSED" or "DEGRADED" or "MODEL_ONLY" or "INVALID"))
            throw new ArgumentException("Invalid mode");
        var uncertainty = frame.GetProperty("uncertainty_available").GetBoolean();
        foreach (var key in new[] { "s_m", "v_mps", "sigma_s_m", "sigma_v_mps" })
        {
            var value = frame.GetProperty(key);
            if (value.ValueKind != JsonValueKind.Null && !double.IsFinite(value.GetDouble()))
                throw new ArgumentException("Nonfinite numerical value");
        }
        if (uncertainty && (frame.GetProperty("sigma_s_m").ValueKind == JsonValueKind.Null ||
                            frame.GetProperty("sigma_v_mps").ValueKind == JsonValueKind.Null))
            throw new ArgumentException("Missing declared uncertainty");
    }
}

sealed class RunStore
{
    readonly string directory;
    readonly string connectionString;
    readonly object gate = new();
    public RunStore(string path)
    {
        directory = Path.GetFullPath(path);
        Directory.CreateDirectory(directory);
        connectionString = new SqliteConnectionStringBuilder
            { DataSource = Path.Combine(directory, "catalog.db"), DefaultTimeout = 10 }.ToString();
        using var db = Open();
        using var cmd = db.CreateCommand();
        cmd.CommandText = """
            PRAGMA journal_mode=WAL;
            CREATE TABLE IF NOT EXISTS runs(id TEXT PRIMARY KEY, metadata TEXT NOT NULL);
            CREATE TABLE IF NOT EXISTS frames(run_id TEXT NOT NULL, seq INTEGER NOT NULL, body TEXT NOT NULL,
                PRIMARY KEY(run_id,seq));
            CREATE TABLE IF NOT EXISTS reports(run_id TEXT PRIMARY KEY, filename TEXT NOT NULL);
            """;
        cmd.ExecuteNonQuery();
    }
    SqliteConnection Open() { var db = new SqliteConnection(connectionString); db.Open(); return db; }
    public void Register(string id, string metadata)
    {
        using var db = Open(); using var cmd = db.CreateCommand();
        cmd.CommandText = "INSERT INTO runs VALUES($id,$body) ON CONFLICT(id) DO UPDATE SET metadata=$body";
        cmd.Parameters.AddWithValue("$id", id); cmd.Parameters.AddWithValue("$body", metadata); cmd.ExecuteNonQuery();
    }
    public string? Get(string id)
    {
        using var db = Open(); using var cmd = db.CreateCommand();
        cmd.CommandText = "SELECT metadata FROM runs WHERE id=$id";
        cmd.Parameters.AddWithValue("$id", id); return cmd.ExecuteScalar() as string;
    }
    public List<JsonElement> List()
    {
        using var db = Open(); using var cmd = db.CreateCommand();
        cmd.CommandText = "SELECT metadata FROM runs ORDER BY rowid DESC LIMIT 200";
        using var reader = cmd.ExecuteReader();
        var result = new List<JsonElement>();
        while (reader.Read()) result.Add(JsonSerializer.Deserialize<JsonElement>(reader.GetString(0)));
        return result;
    }
    public List<JsonElement> Append(string id, JsonElement[] frames)
    {
        lock (gate)
        {
            using var db = Open(); using var transaction = db.BeginTransaction();
            var added = new List<JsonElement>();
            foreach (var frame in frames)
            {
                using var cmd = db.CreateCommand(); cmd.Transaction = transaction;
                cmd.CommandText = "INSERT OR IGNORE INTO frames VALUES($id,$seq,$body)";
                cmd.Parameters.AddWithValue("$id", id);
                cmd.Parameters.AddWithValue("$seq", frame.GetProperty("seq").GetInt64());
                cmd.Parameters.AddWithValue("$body", frame.GetRawText());
                if (cmd.ExecuteNonQuery() > 0) added.Add(frame);
            }
            transaction.Commit();
            return added;
        }
    }
    public List<JsonElement> Frames(string id, long after, int limit)
    {
        using var db = Open(); using var cmd = db.CreateCommand();
        cmd.CommandText = "SELECT body FROM frames WHERE run_id=$id AND seq>$after ORDER BY seq LIMIT $limit";
        cmd.Parameters.AddWithValue("$id", id); cmd.Parameters.AddWithValue("$after", after);
        cmd.Parameters.AddWithValue("$limit", limit);
        using var reader = cmd.ExecuteReader(); var result = new List<JsonElement>();
        while (reader.Read()) result.Add(JsonSerializer.Deserialize<JsonElement>(reader.GetString(0)));
        return result;
    }
    public void SaveReport(string id, string body)
    {
        lock (gate)
        {
            var filename = id + "-" + Guid.NewGuid().ToString("N") + ".json";
            File.WriteAllText(Path.Combine(directory, filename), body);
            using var db = Open(); using var cmd = db.CreateCommand();
            cmd.CommandText = "INSERT INTO reports VALUES($id,$file) ON CONFLICT(run_id) DO UPDATE SET filename=$file";
            cmd.Parameters.AddWithValue("$id", id); cmd.Parameters.AddWithValue("$file", filename);
            cmd.ExecuteNonQuery();
        }
    }
    public string? Report(string id)
    {
        using var db = Open(); using var cmd = db.CreateCommand();
        cmd.CommandText = "SELECT filename FROM reports WHERE run_id=$id";
        cmd.Parameters.AddWithValue("$id", id);
        return cmd.ExecuteScalar() is string file ? File.ReadAllText(Path.Combine(directory, file)) : null;
    }
}
