param(
    [string]$BrokerHost = "127.0.0.1",
    [int]$BrokerPort = 1883,
    [string]$NodeId = "node_01",
    [int]$IntervalMs = 1000,
    [int]$RawWindowSize = 256,
    [int]$RawEveryNMessages = 15
)

$ErrorActionPreference = "Stop"

function New-ProcessedPayload {
    param(
        [string]$NodeId,
        [int]$Tick
    )

    $phase = $Tick / 8.0
    $rms = [Math]::Round(0.10 + 0.03 * [Math]::Sin($phase), 4)
    $pitch = [Math]::Round(0.25 + 0.15 * [Math]::Sin($phase / 2.0), 3)
    $roll = [Math]::Round(0.20 + 0.12 * [Math]::Cos($phase / 2.0), 3)

    return @{
        node_id = $NodeId
        timestamp = [DateTime]::UtcNow.ToString("yyyy-MM-ddTHH:mm:ss.fffZ")
        sampling_rate_hz = 200
        vibration = @{
            rms = $rms
        }
        tilt = @{
            pitch = $pitch
            roll = $roll
            pitch_delta = $pitch
            roll_delta = $roll
        }
        magnetometer = @{
            mag_x = [Math]::Round(10 + 2 * [Math]::Sin($phase), 3)
            mag_y = [Math]::Round(15 + 2 * [Math]::Cos($phase), 3)
            mag_z = [Math]::Round(20 + 1.5 * [Math]::Sin($phase / 3.0), 3)
        }
        connection_status = "online"
    }
}

function New-RawPayload {
    param(
        [string]$NodeId,
        [int]$Tick,
        [int]$RawWindowSize
    )

    $raw = New-Object System.Collections.Generic.List[double]
    for ($i = 0; $i -lt $RawWindowSize; $i++) {
        $v = [Math]::Sin(($i + $Tick) / 8.0) + 0.5 * [Math]::Cos(($i + $Tick) / 16.0)
        $raw.Add([Math]::Round($v, 4))
    }

    return @{
        node_id = $NodeId
        timestamp = [DateTime]::UtcNow.ToString("yyyy-MM-ddTHH:mm:ss.fffZ")
        window_size = $RawWindowSize
        sampling_rate_hz = 200
        raw_accel = $raw
    }
}

$mosquittoPub = Get-Command mosquitto_pub -ErrorAction SilentlyContinue
if (-not $mosquittoPub) {
    throw "mosquitto_pub tidak ditemukan di PATH. Install Mosquitto client atau tambahkan ke PATH."
}

$topicData = "bridge/$NodeId/data"
$topicRaw = "bridge/$NodeId/raw"
$tick = 0

Write-Host "[mock] publish ke mqtt://$BrokerHost`:$BrokerPort"
Write-Host "[mock] topic data: $topicData"
Write-Host "[mock] topic raw : $topicRaw (setiap $RawEveryNMessages pesan)"

while ($true) {
    $tick++

    $processedJson = (New-ProcessedPayload -NodeId $NodeId -Tick $tick) | ConvertTo-Json -Compress -Depth 6
    $processedJson | & $mosquittoPub.Source -h $BrokerHost -p $BrokerPort -t $topicData -l

    if (($tick % $RawEveryNMessages) -eq 0) {
        $rawJson = (New-RawPayload -NodeId $NodeId -Tick $tick -RawWindowSize $RawWindowSize) | ConvertTo-Json -Compress -Depth 6
        $rawJson | & $mosquittoPub.Source -h $BrokerHost -p $BrokerPort -t $topicRaw -l
    }

    Start-Sleep -Milliseconds $IntervalMs
}
