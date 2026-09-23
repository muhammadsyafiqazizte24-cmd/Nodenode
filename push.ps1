#JALANIN NYA KETIK INI DI TERMINAL YAAA .\push.ps1

$ProjectPath = D:\Politeknik Negeri Jakarta\= SHM, SIMON BATAPA\Nodenode

Set-Location $ProjectPath

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "       SHM NODE FW V1 - GIT PUSH        " -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan

Write-Host "`n[1/6] Checking local changes..." -ForegroundColor Yellow

git status --short

Write-Host "`n[2/6] Checking GitHub for updates..." -ForegroundColor Yellow

git fetch origin main

if ($LASTEXITCODE -ne 0) {
    Write-Host "`nERROR: Gagal terhubung ke GitHub." -ForegroundColor Red
    Read-Host "Tekan Enter untuk keluar"
    exit 1
}

$LocalCommit  = git rev-parse HEAD
$RemoteCommit = git rev-parse origin/main

if ($LocalCommit -ne $RemoteCommit) {

    Write-Host "`n[REMOTE UPDATE] Ada perubahan baru dari collaborator!" -ForegroundColor Cyan

    Write-Host "`nCommit dari GitHub:" -ForegroundColor Yellow

    git log "$LocalCommit..origin/main" `
        --pretty=format:"  %h | %an | %ad | %s" `
        --date=format:"%Y-%m-%d %H:%M"

    Write-Host "`n"
    
    Write-Host "[3/6] Updating local repository..." -ForegroundColor Yellow

    git pull --rebase origin main

    if ($LASTEXITCODE -ne 0) {
        Write-Host "`nERROR: Conflict terjadi!" -ForegroundColor Red
        Write-Host "Push dibatalkan. Selesaikan conflict terlebih dahulu." -ForegroundColor Yellow
        Read-Host "Tekan Enter untuk keluar"
        exit 1
    }

    Write-Host "`n✓ Repository berhasil diperbarui." -ForegroundColor Green

} else {

    Write-Host "`n✓ Tidak ada update baru dari collaborator." -ForegroundColor Green
}

Write-Host "`n[4/6] Adding changes..." -ForegroundColor Yellow

git add .

Write-Host "`n[5/6] Creating commit..." -ForegroundColor Yellow

$CommitMessage = Read-Host "Commit message"

if ([string]::IsNullOrWhiteSpace($CommitMessage)) {
    Write-Host "`nERROR: Commit message tidak boleh kosong." -ForegroundColor Red
    Read-Host "Tekan Enter untuk keluar"
    exit 1
}

git commit -m $CommitMessage

if ($LASTEXITCODE -ne 0) {
    Write-Host "`nTidak ada perubahan baru untuk di-commit." -ForegroundColor Yellow
}

Write-Host "`n[6/6] Pushing to GitHub..." -ForegroundColor Yellow

git push origin main

if ($LASTEXITCODE -eq 0) {

    Write-Host "`n========================================" -ForegroundColor Green
    Write-Host "          PUSH BERHASIL!                " -ForegroundColor Green
    Write-Host "========================================" -ForegroundColor Green

} else {

    Write-Host "`n========================================" -ForegroundColor Red
    Write-Host "            PUSH GAGAL!                 " -ForegroundColor Red
    Write-Host "========================================" -ForegroundColor Red
}

Write-Host ""
Read-Host "Tekan Enter untuk keluar"