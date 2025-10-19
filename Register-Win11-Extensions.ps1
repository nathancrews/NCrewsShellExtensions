# Register All Shell Extensions for Windows 11 Compatibility
# Run as Administrator

param(
    [switch]$Unregister,
    [switch]$Test,
    [switch]$ClearCache,
    [switch]$PointCloudOnly,
    [switch]$GLTFOnly,
    [switch]$ResetOpenWith,
    [switch]$ResetUserChoice
)

$ErrorActionPreference = "Continue"

# Elevate to Administrator if needed
if ((-not ([Security.Principal.WindowsPrincipal] [Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole] "Administrator")) -and (-not ($Test -or $ResetOpenWith -or $ResetUserChoice))) {
    Write-Host "Requesting elevation..." -ForegroundColor Yellow
    $psi = New-Object System.Diagnostics.ProcessStartInfo
    $psi.FileName = (Get-Process -Id $PID).Path
    $argList = @('-NoProfile','-ExecutionPolicy','Bypass','-File', $PSCommandPath) + (
        $PSBoundParameters.GetEnumerator() | ForEach-Object {
            if ($_.Value -is [switch] -or $_.Value -eq $true) { '-{0}' -f $_.Key }
            elseif ($_.Value) { '-{0}' -f $_.Key; ('{0}' -f $_.Value) }
        }
    )
    $psi.Arguments = ($argList | ForEach-Object { if ($_ -match "\s") { '"{0}"' -f $_ } else { $_ } }) -join ' '
    $psi.Verb = 'runas'
    try {
        $p = [System.Diagnostics.Process]::Start($psi)
        $p.WaitForExit()
        exit $p.ExitCode
    } catch {
        Write-Error "Elevation cancelled or failed: $_"
        exit 1
    }
}

Write-Host "NCraft Shell Extensions Manager for Windows 11" -ForegroundColor Cyan
Write-Host "===============================================" -ForegroundColor Cyan

$ProjectRoot = Split-Path -Parent $MyInvocation.MyCommand.Path

# Clear cache if requested
if ($ClearCache) {
    Write-Host "Clearing Windows thumbnail cache..." -ForegroundColor Yellow
    
    # Stop Windows Explorer
    Stop-Process -Name "explorer" -Force -ErrorAction SilentlyContinue
    Start-Sleep -Seconds 2
    
    # Clear thumbnail cache
    $thumbCachePattern = "$env:LocalAppData\Microsoft\Windows\Explorer\thumbcache_*.db"
    Get-ChildItem -Path $thumbCachePattern -Force -ErrorAction SilentlyContinue | Remove-Item -Force
    
    # Restart Explorer
    Start-Process "explorer.exe"
    Start-Sleep -Seconds 3
    
    Write-Host "Thumbnail cache cleared successfully." -ForegroundColor Green
}

# Point Cloud Extension
if (-not $GLTFOnly) {
    Write-Host "`n" + "="*60 -ForegroundColor Cyan
    Write-Host "POINT CLOUD SHELL EXTENSION (.las/.laz files)" -ForegroundColor Cyan
    Write-Host "="*60 -ForegroundColor Cyan
    
    $pcParams = @{}
    if ($Unregister) { $pcParams.Unregister = $true }
    if ($Test) { $pcParams.Test = $true }
    if ($ResetOpenWith) { $pcParams.ResetOpenWith = $true }
    if ($ResetUserChoice) { $pcParams.ResetUserChoice = $true }
    
    $pcScriptPath = Join-Path $ProjectRoot "Register-Win11-PointCloud.ps1"
    if (Test-Path $pcScriptPath) {
        & $pcScriptPath @pcParams
    } else {
        Write-Error "Point Cloud registration script not found: $pcScriptPath"
    }
}

# GLTF Extension
if (-not $PointCloudOnly) {
    Write-Host "`n" + "="*60 -ForegroundColor Cyan
    Write-Host "GLTF/GLB SHELL EXTENSION (.gltf/.glb files)" -ForegroundColor Cyan
    Write-Host "="*60 -ForegroundColor Cyan
    
    $gltfParams = @{}
    if ($Unregister) { $gltfParams.Unregister = $true }
    if ($Test) { $gltfParams.Test = $true }
    if ($ResetOpenWith) { $gltfParams.ResetOpenWith = $true }
    if ($ResetUserChoice) { $gltfParams.ResetUserChoice = $true }
    
    $gltfScriptPath = Join-Path $ProjectRoot "Register-Win11-GLTF.ps1"
    if (Test-Path $gltfScriptPath) {
        & $gltfScriptPath @gltfParams
    } else {
        Write-Error "GLTF registration script not found: $gltfScriptPath"
    }
}

# Summary
Write-Host "`n" + "="*60 -ForegroundColor Green
Write-Host "SUMMARY" -ForegroundColor Green
Write-Host "="*60 -ForegroundColor Green

if ($Unregister) {
    Write-Host "Shell extensions have been unregistered." -ForegroundColor Yellow
    Write-Host "You may need to restart Windows Explorer or reboot." -ForegroundColor Yellow
} elseif ($Test) {
    Write-Host "Registration test completed. Check output above for any issues." -ForegroundColor Cyan
} else {
    Write-Host "Shell extensions have been registered for Windows 11." -ForegroundColor Green
    Write-Host "`nRecommended next steps:" -ForegroundColor Yellow
    Write-Host "1. Build the project in Release|x64 mode" -ForegroundColor Yellow
    Write-Host "2. Run: .\Register-Win11-Extensions.ps1 -ClearCache" -ForegroundColor Yellow
    Write-Host "3. Run: .\Register-Win11-Extensions.ps1 -Test" -ForegroundColor Yellow
    Write-Host "4. Test with sample files in File Explorer" -ForegroundColor Yellow
    Write-Host "`nTroubleshooting:" -ForegroundColor Yellow
    Write-Host "• If extensions don't appear, restart Windows Explorer" -ForegroundColor Yellow
    Write-Host "• Check that DLL files exist in build/Release/ directories" -ForegroundColor Yellow
    Write-Host "• Run scripts as Administrator" -ForegroundColor Yellow
    Write-Host "• For debug info, check log files in %TEMP% directory" -ForegroundColor Yellow
}

Write-Host "`nDone!" -ForegroundColor Green