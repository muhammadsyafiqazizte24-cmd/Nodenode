$ProjectPath = "C:\Users\Syafiq\Documents\College\SIMON BATAPA\NodeNode"

Set-Location $ProjectPath

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "       SHM NODE FW V1 - GIT PULL        " -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan

# ========================================
# 1. Check repository
# ========================================

Write-Host ""
Write-Host "[1/3] Checking repository..." -ForegroundColor Yellow

git status --short

if ($LASTEXITCODE -ne 0) {
    Write-Host ""
    Write-Host "ERROR: Folder ini bukan Git repository!" -ForegroundColor Red
    Read-Host "Tekan Enter untuk keluar"
    exit 1
}

# ========================================
# 2. Fetch GitHub
# ========================================

Write-Host ""
Write-Host "[2/3] Checking GitHub for updates..." -ForegroundColor Yellow

git fetch origin main

if ($LASTEXITCODE -ne 0) {
    Write-Host ""
    Write-Host "ERROR: Gagal terhubung ke GitHub." -ForegroundColor Red
    Read-Host "Tekan Enter untuk keluar"
    exit 1
}

# Get commit hashes
$LocalCommit = git rev-parse HEAD
$RemoteCommit = git rev-parse origin/main

# ========================================
# Compare local vs remote
# ========================================

if ($LocalCommit -eq $RemoteCommit) {

    Write-Host ""
    Write-Host "Repository sudah up-to-date." -ForegroundColor Green

}
else {

    Write-Host ""
    Write-Host "[REMOTE UPDATE] Ada perubahan baru dari GitHub!" -ForegroundColor Cyan

    Write-Host ""
    Write-Host "Commit baru:" -ForegroundColor Yellow

    git log "$LocalCommit..origin/main" --oneline --decorate

    # ========================================
    # Pull changes
    # ========================================

    Write-Host ""
    Write-Host "Pulling changes..." -ForegroundColor Yellow

    git pull --rebase origin main

    $PullExitCode = $LASTEXITCODE

    if ($PullExitCode -ne 0) {

        Write-Host ""
        Write-Host "ERROR: Pull gagal atau terjadi conflict!" -ForegroundColor Red
        Write-Host "Silakan selesaikan conflict terlebih dahulu." -ForegroundColor Yellow

        Read-Host "Tekan Enter untuk keluar"
        exit 1
    }

    Write-Host ""
    Write-Host "Update berhasil diambil." -ForegroundColor Green
}

# ========================================
# 3. Final status
# ========================================

Write-Host ""
Write-Host "[3/3] Current status:" -ForegroundColor Yellow

git status

Write-Host ""
Write-Host "========================================" -ForegroundColor Green
Write-Host "             PULL SELESAI               " -ForegroundColor Green
Write-Host "========================================" -ForegroundColor Green

Write-Host ""
Read-Host "Tekan Enter untuk keluar"