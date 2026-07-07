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
$InstalledDllCandidates = @(
    "C:\Program Files\NCrews Software\NCrews GLTF Shell Extension 2.0\ModelShellExtension.dll",
    "C:\Program Files\NCrews Software\NCrews GLTF Shell Extension\ModelShellExtension.dll"
)
$ThumbnailProviderKeys = @(
    "{E357FCCD-A995-4576-B01F-234630154E96}",
    "{BB2E617C-0920-11D1-9A0B-00C04FC2D6C1}"
)
$ThumbnailExtensions = @('.glb', '.stl', '.obj', '.3mf')
$ThumbnailAutoFileClasses = @{
    '.glb' = 'glb_auto_file'
    '.stl' = 'stl_auto_file'
    '.obj' = 'obj_auto_file'
    '.3mf' = '3mf_auto_file'
}

function Resolve-InstalledDllPath {
    foreach ($candidate in $InstalledDllCandidates) {
        if (Test-Path $candidate) {
            return $candidate
        }
    }

    $installedRoot = Join-Path $env:ProgramFiles "NCrews Software"
    if (Test-Path $installedRoot) {
        $dllCandidate = Get-ChildItem -Path $installedRoot -Filter "ModelShellExtension.dll" -Recurse -File -ErrorAction SilentlyContinue |
            Sort-Object LastWriteTime -Descending |
            Select-Object -First 1
        if ($dllCandidate) {
            return $dllCandidate.FullName
        }
    }

    return $null
}

$ActualDllPath = Resolve-InstalledDllPath
if ($ActualDllPath) {
    Write-Host "Resolved shell extension DLL: $ActualDllPath" -ForegroundColor Green
} else {
    Write-Warning "Could not resolve ModelShellExtension.dll in expected install locations."
}

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
    if (-not $ActualDllPath) {
        Write-Error "ModelShellExtension.dll could not be found. Please install the NCrews GLTF Shell Extension first."
        exit 1
    }
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

            foreach ($thumbExt in $ThumbnailExtensions) {
                foreach ($providerKey in $ThumbnailProviderKeys) {
                    $thumbRegPath = "$root\Software\Classes\$thumbExt\ShellEx\$providerKey"
                    if (-not (Test-Path $thumbRegPath)) { New-Item -Path $thumbRegPath -Force | Out-Null }
                    Set-ItemProperty -Path $thumbRegPath -Name "(default)" -Value $ThumbnailGUID
                }

                if ($ThumbnailAutoFileClasses.ContainsKey($thumbExt)) {
                    $autoClass = $ThumbnailAutoFileClasses[$thumbExt]
                    foreach ($providerKey in $ThumbnailProviderKeys) {
                        $autoThumbRegPath = "$root\Software\Classes\$autoClass\ShellEx\$providerKey"
                        if (-not (Test-Path $autoThumbRegPath)) { New-Item -Path $autoThumbRegPath -Force | Out-Null }
                        Set-ItemProperty -Path $autoThumbRegPath -Name "(default)" -Value $ThumbnailGUID
                    }
                }
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
        New-ItemProperty -Path $approved -Name $ThumbnailGUID -PropertyType String -Value "Model Shell Extension Thumbnail" -Force | Out-Null
        
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
    if ($ActualDllPath -and (Test-Path $ActualDllPath)) {
        $result = & "$env:WINDIR\System32\regsvr32.exe" "/u" $ActualDllPath 2>&1
        if ($LASTEXITCODE -eq 0) {
            Write-Host "DLL unregistered successfully." -ForegroundColor Green
        } else {
            Write-Warning "Failed to unregister DLL. Error code: $LASTEXITCODE`nOutput: $result"
        }
    } else {
        Write-Warning "Skipping regsvr32 /u because ModelShellExtension.dll could not be resolved."
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
            foreach ($thumbExt in $ThumbnailExtensions) {
                foreach ($providerKey in $ThumbnailProviderKeys) {
                    Remove-Item -Path "$root\Software\Classes\$thumbExt\ShellEx\$providerKey" -Recurse -Force -ErrorAction SilentlyContinue
                }
                if ($ThumbnailAutoFileClasses.ContainsKey($thumbExt)) {
                    $autoClass = $ThumbnailAutoFileClasses[$thumbExt]
                    foreach ($providerKey in $ThumbnailProviderKeys) {
                        Remove-Item -Path "$root\Software\Classes\$autoClass\ShellEx\$providerKey" -Recurse -Force -ErrorAction SilentlyContinue
                    }
                }
            }
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

    if ($ActualDllPath) {
        Write-Host "DLL location: $ActualDllPath" -ForegroundColor Green
    } else {
        Write-Host "DLL location: NOT FOUND" -ForegroundColor Red
    }

    foreach ($root in @('HKCU:', 'HKLM:')) {
        Write-Host ""
        Write-Host "[$root] Registry verification" -ForegroundColor Cyan

        foreach ($guid in @("{0C6D56CF-1C57-4BE1-8736-5A3D02A68187}", "{CB7B16EE-63F0-498A-AD7E-857BD1B560C6}")) {
            $clsidPath = "$root\Software\Classes\CLSID\$guid\InprocServer32"
            if (Test-Path $clsidPath) {
                $dll = (Get-ItemProperty -Path $clsidPath -ErrorAction SilentlyContinue)."(default)"
                $threading = (Get-ItemProperty -Path $clsidPath -ErrorAction SilentlyContinue)."ThreadingModel"
                Write-Host "CLSID $guid => dll=$dll threading=$threading" -ForegroundColor Green
            } else {
                Write-Host "Missing CLSID registration: $clsidPath" -ForegroundColor Red
            }
        }

        foreach ($ext in @('.glb', '.gltf', '.stl', '.obj', '.3mf')) {
            $ctxPath = "$root\Software\Classes\$ext\ShellEx\ContextMenuHandlers\ModelShellExtension"
            if (Test-Path $ctxPath) {
                $ctxVal = (Get-ItemProperty -Path $ctxPath -ErrorAction SilentlyContinue)."(default)"
                Write-Host "ContextMenu $ext => $ctxVal" -ForegroundColor Green
            } else {
                Write-Host "Missing context menu handler: $ctxPath" -ForegroundColor Red
            }

            $sfaPath = "$root\Software\Classes\SystemFileAssociations\$ext\ShellEx\ContextMenuHandlers\ModelShellExtension"
            if (Test-Path $sfaPath) {
                $sfaVal = (Get-ItemProperty -Path $sfaPath -ErrorAction SilentlyContinue)."(default)"
                Write-Host "SystemFileAssociations $ext => $sfaVal" -ForegroundColor Green
            } else {
                Write-Host "Missing SystemFileAssociations handler: $sfaPath" -ForegroundColor Red
            }

            if ($ThumbnailExtensions -contains $ext) {
                foreach ($providerKey in $ThumbnailProviderKeys) {
                    $thumbPath = "$root\Software\Classes\$ext\ShellEx\$providerKey"
                    if (Test-Path $thumbPath) {
                        $thumbVal = (Get-ItemProperty -Path $thumbPath -ErrorAction SilentlyContinue)."(default)"
                        Write-Host "Thumbnail $ext ($providerKey) => $thumbVal" -ForegroundColor Green
                    } else {
                        Write-Host "Missing thumbnail provider: $thumbPath" -ForegroundColor Red
                    }
                }

                if ($ThumbnailAutoFileClasses.ContainsKey($ext)) {
                    $autoClass = $ThumbnailAutoFileClasses[$ext]
                    foreach ($providerKey in $ThumbnailProviderKeys) {
                        $autoThumbPath = "$root\Software\Classes\$autoClass\ShellEx\$providerKey"
                        if (Test-Path $autoThumbPath) {
                            $autoThumbVal = (Get-ItemProperty -Path $autoThumbPath -ErrorAction SilentlyContinue)."(default)"
                            Write-Host "Thumbnail $autoClass ($providerKey) => $autoThumbVal" -ForegroundColor Green
                        } else {
                            Write-Host "Missing thumbnail provider: $autoThumbPath" -ForegroundColor Red
                        }
                    }
                }
            }
        }
    }
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
    Write-Host "2. Run: .\Register-Win11-GLTF-2.0.ps1 -Test" -ForegroundColor Yellow
    Write-Host "3. Test with actual .glb/.gltf/.stl/.obj/.3mf files" -ForegroundColor Yellow
}

if ($ClearCache) {
    Clear-ThumbnailCache
}

Show-DiagnosticLogInfo

Write-Host "`nDone!" -ForegroundColor Green
if ($Host.Name -eq "ConsoleHost" -and -not $Test)
{
    Write-Host "`nPress any key to exit..." -ForegroundColor Cyan
    $null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")
}
