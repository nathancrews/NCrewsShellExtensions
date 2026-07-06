# Register GLTF Shell Extension for Windows 11 Compatibility
# Run as Administrator

param(
    [switch]$Unregister,
    [switch]$Test,
    [switch]$ClearCache,
    [switch]$ResetOpenWith
)

$ErrorActionPreference = "Continue"

# Paths
$InstalledDllPath = "C:\Program Files\NCrews Software\NCrews GLTF Shell Extension\ModelShellExtension.dll"

# Verify the installed DLL exists
if (-not (Test-Path $InstalledDllPath)) {
    Write-Error "ModelShellExtension.dll not found at $InstalledDllPath`nPlease ensure the NCrews GLTF Shell Extension is installed."
    exit 1
}

$ActualDllPath = $InstalledDllPath

function Clear-ThumbnailCache {
    Write-Host "Clearing Windows thumbnail cache..." -ForegroundColor Yellow
    
    # Stop Windows Explorer
    Stop-Process -Name "explorer" -Force -ErrorAction SilentlyContinue
    Start-Sleep -Seconds 3
    
    # Wait for Explorer process to fully terminate
    $timeout = 0
    while ((Get-Process -Name "explorer" -ErrorAction SilentlyContinue) -and $timeout -lt 10) {
        Start-Sleep -Seconds 1
        $timeout++
    }
    
    # Clear thumbnail cache
    $thumbCachePattern = "$env:LocalAppData\Microsoft\Windows\Explorer\thumbcache_*.db"
    Get-ChildItem -Path $thumbCachePattern -Force -ErrorAction SilentlyContinue | Remove-Item -Force -ErrorAction SilentlyContinue
    
    # Restart Explorer
    Start-Process "explorer.exe"
    Start-Sleep -Seconds 3
    
    Write-Host "Thumbnail cache cleared successfully." -ForegroundColor Green
}

function Show-DiagnosticLogInfo {
    $modelShellLogPath = Join-Path $env:TEMP "ModelShellExtension.log"
    $open3dDebugLogPath = Join-Path $env:TEMP "Open3D_GLB_Debug.log"
    $desktopPath = Join-Path $env:USERPROFILE "Desktop"

    Write-Host "`nDiagnostic logs (share these with support):" -ForegroundColor Cyan
    foreach ($logPath in @($modelShellLogPath, $open3dDebugLogPath)) {
        if (Test-Path $logPath) {
            $logInfo = Get-Item $logPath -ErrorAction SilentlyContinue
            if ($logInfo) {
                Write-Host ("- {0} (exists, {1} bytes, updated {2})" -f $logPath, $logInfo.Length, $logInfo.LastWriteTime) -ForegroundColor Green
            } else {
                Write-Host ("- {0} (exists)" -f $logPath) -ForegroundColor Green
            }
        } else {
            Write-Host ("- {0} (not present yet)" -f $logPath) -ForegroundColor DarkYellow
        }
    }

    Write-Host "Copy any existing log file from the paths above and send it to support." -ForegroundColor Yellow
    Write-Host ("Example: Copy-Item -Path ""{0}"" -Destination ""{1}"" -Force" -f $modelShellLogPath, $desktopPath) -ForegroundColor Yellow
}

function Add-HandlerToProgId {
    param(
        [Parameter(Mandatory)] [string]$Ext,
        [Parameter(Mandatory)] [string]$HandlerName,
        [Parameter(Mandatory)] [string]$Guid
    )
    $roots = @('HKCU:','HKLM:')
    $hkcrDefault = (Get-ItemProperty -Path ("Registry::HKEY_CLASSES_ROOT\\$Ext") -ErrorAction SilentlyContinue)."(default)"
    $userChoice = (Get-ItemProperty -Path ("HKCU:\\Software\\Microsoft\\Windows\\CurrentVersion\\Explorer\\FileExts\\$Ext\\UserChoice") -ErrorAction SilentlyContinue).ProgId
    $progId = if ($userChoice) { $userChoice } else { $hkcrDefault }
    if (-not $progId) { return }
    foreach ($root in $roots) {
        $regPath = "$root\Software\Classes\$progId\ShellEx\ContextMenuHandlers\$HandlerName"
        if (-not (Test-Path $regPath)) { New-Item -Path $regPath -Force | Out-Null }
        Set-ItemProperty -Path $regPath -Name "(default)" -Value $Guid -ErrorAction SilentlyContinue
    }
}

function Register-Extension {
    # Register the DLL (regsvr32 shows its own success dialog)
    & "$env:WINDIR\System32\regsvr32.exe" /s $ActualDllPath *>$null
    
    # Additional Windows 11 registry entries
    Write-Host "Configuring Windows 11 registry entries..." -ForegroundColor Yellow
    
    # GUIDs from code (inc/ModelShellExtension/*GUID.h)
    $MenuGUID = "{CB7B16EE-63F0-498A-AD7E-857BD1B560C6}"
    $ThumbnailGUID = "{0C6D56CF-1C57-4BE1-8736-5A3D02A68187}"
    
    try {
        # Ensure proper thumbnail provider registration
        foreach ($root in @('HKCU:','HKLM:')) {
            $regPath = "$root\Software\Classes\.glb\ShellEx\{E357FCCD-A995-4576-B01F-234630154E96}"
            if (-not (Test-Path $regPath)) { New-Item -Path $regPath -Force | Out-Null }
            Set-ItemProperty -Path $regPath -Name "(default)" -Value $ThumbnailGUID

            # Add context menu handler for GLB files
            $regPath = "$root\Software\Classes\.glb\ShellEx\ContextMenuHandlers\ModelShellExtension"
            if (-not (Test-Path $regPath)) { New-Item -Path $regPath -Force | Out-Null }
            Set-ItemProperty -Path $regPath -Name "(default)" -Value $MenuGUID
            # Also SystemFileAssociations to force per-file menus
            $regPath = "$root\Software\Classes\SystemFileAssociations\.glb\ShellEx\ContextMenuHandlers\ModelShellExtension"
            if (-not (Test-Path $regPath)) { New-Item -Path $regPath -Force | Out-Null }
            Set-ItemProperty -Path $regPath -Name "(default)" -Value $MenuGUID

            # Add for GLTF files as well
            $regPath = "$root\Software\Classes\.gltf\ShellEx\ContextMenuHandlers\ModelShellExtension"
            if (-not (Test-Path $regPath)) { New-Item -Path $regPath -Force | Out-Null }
            Set-ItemProperty -Path $regPath -Name "(default)" -Value $MenuGUID
            $regPath = "$root\Software\Classes\SystemFileAssociations\.gltf\ShellEx\ContextMenuHandlers\ModelShellExtension"
            if (-not (Test-Path $regPath)) { New-Item -Path $regPath -Force | Out-Null }
            Set-ItemProperty -Path $regPath -Name "(default)" -Value $MenuGUID

            # Add for STL files
            $regPath = "$root\Software\Classes\.stl\ShellEx\ContextMenuHandlers\ModelShellExtension"
            if (-not (Test-Path $regPath)) { New-Item -Path $regPath -Force | Out-Null }
            Set-ItemProperty -Path $regPath -Name "(default)" -Value $MenuGUID
            $regPath = "$root\Software\Classes\SystemFileAssociations\.stl\ShellEx\ContextMenuHandlers\ModelShellExtension"
            if (-not (Test-Path $regPath)) { New-Item -Path $regPath -Force | Out-Null }
            Set-ItemProperty -Path $regPath -Name "(default)" -Value $MenuGUID

            # Ensure STL thumbnail provider registration
            $regPath = "$root\Software\Classes\.stl\ShellEx\{E357FCCD-A995-4576-B01F-234630154E96}"
            if (-not (Test-Path $regPath)) { New-Item -Path $regPath -Force | Out-Null }
            Set-ItemProperty -Path $regPath -Name "(default)" -Value $ThumbnailGUID

            foreach ($ext in @('.obj', '.3mf')) {
                $regPath = "$root\Software\Classes\$ext\ShellEx\ContextMenuHandlers\ModelShellExtension"
                if (-not (Test-Path $regPath)) { New-Item -Path $regPath -Force | Out-Null }
                Set-ItemProperty -Path $regPath -Name "(default)" -Value $MenuGUID

                $regPath = "$root\Software\Classes\SystemFileAssociations\$ext\ShellEx\ContextMenuHandlers\ModelShellExtension"
                if (-not (Test-Path $regPath)) { New-Item -Path $regPath -Force | Out-Null }
                Set-ItemProperty -Path $regPath -Name "(default)" -Value $MenuGUID

                $regPath = "$root\Software\Classes\$ext\ShellEx\{E357FCCD-A995-4576-B01F-234630154E96}"
                if (-not (Test-Path $regPath)) { New-Item -Path $regPath -Force | Out-Null }
                Set-ItemProperty -Path $regPath -Name "(default)" -Value $ThumbnailGUID
            }
        }

        # Also bind to current ProgID
        Add-HandlerToProgId -Ext '.glb' -HandlerName 'ModelShellExtension' -Guid $MenuGUID
        Add-HandlerToProgId -Ext '.gltf' -HandlerName 'ModelShellExtension' -Guid $MenuGUID
        Add-HandlerToProgId -Ext '.stl' -HandlerName 'ModelShellExtension' -Guid $MenuGUID
        Add-HandlerToProgId -Ext '.obj' -HandlerName 'ModelShellExtension' -Guid $MenuGUID
        Add-HandlerToProgId -Ext '.3mf' -HandlerName 'ModelShellExtension' -Guid $MenuGUID
        
        # Add to approved shell extensions (system-wide)
        $approved = "HKLM:\SOFTWARE\Microsoft\Windows\CurrentVersion\Shell Extensions\Approved"
        if (-not (Test-Path $approved)) { New-Item -Path $approved -Force | Out-Null }
        New-ItemProperty -Path $approved -Name $MenuGUID -PropertyType String -Value "Model Shell Extension" -Force | Out-Null
        
        Write-Host "Registry configured successfully." -ForegroundColor Green
        
    } catch {
        Write-Error "Failed to configure registry entries: $_"
    }
    
    # Notify shell of changes
    Write-Host "Refreshing Windows Shell..." -ForegroundColor Yellow
    
    $code = @'
using System;
using System.Runtime.InteropServices;
public class Shell32 {
    [DllImport("shell32.dll")]
    public static extern void SHChangeNotify(uint wEventId, uint uFlags, IntPtr dwItem1, IntPtr dwItem2);
}
'@
    Add-Type -TypeDefinition $code
    
    [Shell32]::SHChangeNotify(0x08000000, 0x0000, [IntPtr]::Zero, [IntPtr]::Zero)  # SHCNE_ASSOCCHANGED
}

function Unregister-Extension {
    Write-Host "Unregistering GLTF Shell Extension..." -ForegroundColor Yellow
    
    # Unregister the DLL
    $result = & "$env:WINDIR\System32\regsvr32.exe" "/u" $ActualDllPath 2>&1
    if ($LASTEXITCODE -eq 0) {
        Write-Host "DLL unregistered successfully." -ForegroundColor Green
    } else {
        Write-Warning "Failed to unregister DLL. Error code: $LASTEXITCODE`nOutput: $result"
    }
    
    # Remove registry entries
    try {
        foreach ($root in @('HKCU:','HKLM:')) {
            Remove-Item -Path "$root\Software\Classes\.glb\ShellEx\ContextMenuHandlers\ModelShellExtension" -Recurse -Force -ErrorAction SilentlyContinue
            Remove-Item -Path "$root\Software\Classes\SystemFileAssociations\.glb\ShellEx\ContextMenuHandlers\ModelShellExtension" -Recurse -Force -ErrorAction SilentlyContinue
            Remove-Item -Path "$root\Software\Classes\.gltf\ShellEx\ContextMenuHandlers\ModelShellExtension" -Recurse -Force -ErrorAction SilentlyContinue
            Remove-Item -Path "$root\Software\Classes\SystemFileAssociations\.gltf\ShellEx\ContextMenuHandlers\ModelShellExtension" -Recurse -Force -ErrorAction SilentlyContinue
            Remove-Item -Path "$root\Software\Classes\.stl\ShellEx\ContextMenuHandlers\ModelShellExtension" -Recurse -Force -ErrorAction SilentlyContinue
            Remove-Item -Path "$root\Software\Classes\SystemFileAssociations\.stl\ShellEx\ContextMenuHandlers\ModelShellExtension" -Recurse -Force -ErrorAction SilentlyContinue
            Remove-Item -Path "$root\Software\Classes\.glb\ShellEx\{E357FCCD-A995-4576-B01F-234630154E96}" -Recurse -Force -ErrorAction SilentlyContinue
            Remove-Item -Path "$root\Software\Classes\.stl\ShellEx\{E357FCCD-A995-4576-B01F-234630154E96}" -Recurse -Force -ErrorAction SilentlyContinue
            Remove-Item -Path "$root\Software\Classes\.obj\ShellEx\ContextMenuHandlers\ModelShellExtension" -Recurse -Force -ErrorAction SilentlyContinue
            Remove-Item -Path "$root\Software\Classes\SystemFileAssociations\.obj\ShellEx\ContextMenuHandlers\ModelShellExtension" -Recurse -Force -ErrorAction SilentlyContinue
            Remove-Item -Path "$root\Software\Classes\.3mf\ShellEx\ContextMenuHandlers\ModelShellExtension" -Recurse -Force -ErrorAction SilentlyContinue
            Remove-Item -Path "$root\Software\Classes\SystemFileAssociations\.3mf\ShellEx\ContextMenuHandlers\ModelShellExtension" -Recurse -Force -ErrorAction SilentlyContinue
            Remove-Item -Path "$root\Software\Classes\.obj\ShellEx\{E357FCCD-A995-4576-B01F-234630154E96}" -Recurse -Force -ErrorAction SilentlyContinue
            Remove-Item -Path "$root\Software\Classes\.3mf\ShellEx\{E357FCCD-A995-4576-B01F-234630154E96}" -Recurse -Force -ErrorAction SilentlyContinue
            foreach($ext in '.glb','.gltf','.stl','.obj','.3mf'){
                $hkcrDefault = (Get-ItemProperty -Path ("Registry::HKEY_CLASSES_ROOT\\$ext") -ErrorAction SilentlyContinue)."(default)"
                $userChoice = (Get-ItemProperty -Path ("HKCU:\\Software\\Microsoft\\Windows\\CurrentVersion\\Explorer\\FileExts\\$ext\\UserChoice") -ErrorAction SilentlyContinue).ProgId
                $progId = if ($userChoice) { $userChoice } else { $hkcrDefault }
                if ($progId) { Remove-Item -Path "$root\Software\Classes\$progId\ShellEx\ContextMenuHandlers\ModelShellExtension" -Recurse -Force -ErrorAction SilentlyContinue }
            }
        }
        Write-Host "Registry entries removed." -ForegroundColor Green
    } catch {
        Write-Warning "Some registry entries could not be removed: $_"
    }
}

function Reset-OpenWithList {
    param([string[]]$Extensions)
    Write-Host "Resetting OpenWithList..." -ForegroundColor Yellow
    foreach ($ext in $Extensions) {
        $owl = "HKCU:\SOFTWARE\Microsoft\Windows\CurrentVersion\Explorer\FileExts\$ext\OpenWithList"
        if (Test-Path $owl) {
            Remove-Item -Path $owl -Recurse -Force -ErrorAction SilentlyContinue
            Write-Host "Cleared OpenWithList for $ext" -ForegroundColor Green
        }
    }
}

function Test-Extension {
    Write-Host "Testing GLTF Shell Extension..." -ForegroundColor Yellow
    
    Write-Host "DLL location: $ActualDllPath" -ForegroundColor Green
    
    # Check registry entries
    $regPaths = @(
        "HKCU:\Software\Classes\.glb\ShellEx\{E357FCCD-A995-4576-B01F-234630154E96}",
        "HKCU:\Software\Classes\.glb\ShellEx\ContextMenuHandlers\ModelShellExtension",
        "HKCU:\Software\Classes\SystemFileAssociations\.glb\ShellEx\ContextMenuHandlers\ModelShellExtension",
        "HKCU:\Software\Classes\.gltf\ShellEx\ContextMenuHandlers\ModelShellExtension",
        "HKCU:\Software\Classes\SystemFileAssociations\.gltf\ShellEx\ContextMenuHandlers\ModelShellExtension",
        "HKCU:\Software\Classes\.stl\ShellEx\{E357FCCD-A995-4576-B01F-234630154E96}",
        "HKCU:\Software\Classes\.stl\ShellEx\ContextMenuHandlers\ModelShellExtension",
        "HKCU:\Software\Classes\SystemFileAssociations\.stl\ShellEx\ContextMenuHandlers\ModelShellExtension",
        "HKCU:\Software\Classes\.obj\ShellEx\{E357FCCD-A995-4576-B01F-234630154E96}",
        "HKCU:\Software\Classes\.obj\ShellEx\ContextMenuHandlers\ModelShellExtension",
        "HKCU:\Software\Classes\SystemFileAssociations\.obj\ShellEx\ContextMenuHandlers\ModelShellExtension",
        "HKCU:\Software\Classes\.3mf\ShellEx\{E357FCCD-A995-4576-B01F-234630154E96}",
        "HKCU:\Software\Classes\.3mf\ShellEx\ContextMenuHandlers\ModelShellExtension",
        "HKCU:\Software\Classes\SystemFileAssociations\.3mf\ShellEx\ContextMenuHandlers\ModelShellExtension",
        "HKLM:\Software\Classes\.glb\ShellEx\{E357FCCD-A995-4576-B01F-234630154E96}",
        "HKLM:\Software\Classes\.glb\ShellEx\ContextMenuHandlers\ModelShellExtension",
        "HKLM:\Software\Classes\SystemFileAssociations\.glb\ShellEx\ContextMenuHandlers\ModelShellExtension",
        "HKLM:\Software\Classes\.gltf\ShellEx\ContextMenuHandlers\ModelShellExtension",
        "HKLM:\Software\Classes\SystemFileAssociations\.gltf\ShellEx\ContextMenuHandlers\ModelShellExtension",
        "HKLM:\Software\Classes\.stl\ShellEx\{E357FCCD-A995-4576-B01F-234630154E96}",
        "HKLM:\Software\Classes\.stl\ShellEx\ContextMenuHandlers\ModelShellExtension",
        "HKLM:\Software\Classes\SystemFileAssociations\.stl\ShellEx\ContextMenuHandlers\ModelShellExtension",
        "HKLM:\Software\Classes\.obj\ShellEx\{E357FCCD-A995-4576-B01F-234630154E96}",
        "HKLM:\Software\Classes\.obj\ShellEx\ContextMenuHandlers\ModelShellExtension",
        "HKLM:\Software\Classes\SystemFileAssociations\.obj\ShellEx\ContextMenuHandlers\ModelShellExtension",
        "HKLM:\Software\Classes\.3mf\ShellEx\{E357FCCD-A995-4576-B01F-234630154E96}",
        "HKLM:\Software\Classes\.3mf\ShellEx\ContextMenuHandlers\ModelShellExtension",
        "HKLM:\Software\Classes\SystemFileAssociations\.3mf\ShellEx\ContextMenuHandlers\ModelShellExtension"
    )
    
    foreach ($regPath in $regPaths) {
        if (Test-Path $regPath) {
            $value = (Get-ItemProperty -Path $regPath -Name "(default)" -ErrorAction SilentlyContinue)."(default)"
            Write-Host "Registry entry found: $regPath = $value" -ForegroundColor Green
        } else {
            Write-Host "Missing registry entry: $regPath" -ForegroundColor Red
        }
    }
    
    Write-Host "`nTo test thumbnails:" -ForegroundColor Cyan
    Write-Host "1. Create or copy a .glb file to your desktop" -ForegroundColor Cyan
    Write-Host "2. Switch to thumbnail view in File Explorer" -ForegroundColor Cyan
    Write-Host "3. Check if custom thumbnail appears" -ForegroundColor Cyan
    Write-Host "4. Right-click the file to see context menu" -ForegroundColor Cyan
}

# Main execution
# Elevate to Administrator if needed
if ((-not ([Security.Principal.WindowsPrincipal] [Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole] "Administrator")) -and (-not ($Test -or $ResetOpenWith))) {
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

Write-Host "GLTF Shell Extension Manager for Windows 11" -ForegroundColor Cyan
Write-Host "============================================" -ForegroundColor Cyan

if ($Unregister) {
    Unregister-Extension
} elseif ($Test) {
    Test-Extension
    if ($ResetOpenWith) { Reset-OpenWithList -Extensions @('.glb','.gltf','.stl','.obj','.3mf') }
} elseif ($ResetOpenWith) {
    Reset-OpenWithList -Extensions @('.glb','.gltf','.stl','.obj','.3mf')
} else {
    Register-Extension
    # Automatically clear cache after registration
    Clear-ThumbnailCache
    Write-Host "`nRecommended next steps:" -ForegroundColor Yellow
    Write-Host "1. Refresh File Explorer or open a new window" -ForegroundColor Yellow
    Write-Host "2. Run: .\Register-Win11-GLTF.ps1 -Test" -ForegroundColor Yellow
    Write-Host "3. Test with actual .glb/.gltf/.stl/.obj/.3mf files" -ForegroundColor Yellow
}

if ($ClearCache) {
    Clear-ThumbnailCache
}

Show-DiagnosticLogInfo

Write-Host "`nDone!" -ForegroundColor Green
Write-Host "`nPress any key to exit..." -ForegroundColor Cyan
$null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")
