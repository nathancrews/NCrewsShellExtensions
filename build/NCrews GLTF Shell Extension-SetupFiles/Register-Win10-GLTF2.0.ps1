# Register-Win10-GLTF.ps1
# Registers model file shell integration on Windows 10 for:
# .glb, .gltf, .stl, .obj, .3mf
#
# Root cause: Windows 10 writes a UserChoice ProgId when a file type is opened with any app.
# If that ProgId points to a UWP/Store app, Windows 10 suppresses third-party context menu
# handlers for that file type entirely. Clearing these locks and registering under
# SystemFileAssociations resolves the issue persistently.
#
# Usage:
#   .\Register-Win10-GLTF.ps1             - Apply registration (requires elevation)
#   .\Register-Win10-GLTF.ps1 -Test       - Diagnose current registry state (no elevation needed)
#   .\Register-Win10-GLTF.ps1 -Unregister - Remove registration entries

param(
    [switch]$Test,
    [switch]$Unregister
)

$ErrorActionPreference = "Continue"

$InstalledDllPath       = "C:\Program Files\NCrews Software\NCrews GLTF Shell Extension\ModelShellExtension.dll"
$MenuGUID               = "{CB7B16EE-63F0-498A-AD7E-857BD1B560C6}"
$ThumbnailGUID          = "{0C6D56CF-1C57-4BE1-8736-5A3D02A68187}"
$ThumbProviderKey       = "{E357FCCD-A995-4576-B01F-234630154E96}"
$ContextMenuExtensions  = @('.glb', '.gltf', '.stl', '.obj', '.3mf')
$ThumbnailExtensions    = @('.glb', '.stl', '.obj', '.3mf')

# Warn if this is not Windows 10
$osBuild = [System.Environment]::OSVersion.Version.Build
if ($osBuild -ge 22000) {
    Write-Warning "This script targets Windows 10. Detected Windows 11 (build $osBuild)."
    Write-Warning "Use Register-Win11-GLTF.ps1 for Windows 11 instead."
}

function Register-ThumbnailAssociations {
    param([string]$Ext)

    foreach ($root in @('HKCU:', 'HKLM:')) {
        $regPath = "$root\Software\Classes\$Ext\ShellEx\$ThumbProviderKey"
        try {
            if (-not (Test-Path $regPath)) { New-Item -Path $regPath -Force | Out-Null }
            Set-ItemProperty -Path $regPath -Name "(default)" -Value $ThumbnailGUID -ErrorAction Stop
            Write-Host "    Registered thumbnail provider for $Ext under $root" -ForegroundColor Green
        } catch {
            Write-Host "    Skipped thumbnail key under $root (no write access - expected without admin): $_" -ForegroundColor Yellow
        }
    }
}

# ---------------------------------------------------------------------------
# Utility: notify the shell of association changes
# ---------------------------------------------------------------------------
function Invoke-ShellRefresh {
    $code = @'
using System;
using System.Runtime.InteropServices;
public class Shell32Win10Fix {
    [DllImport("shell32.dll")]
    public static extern void SHChangeNotify(uint wEventId, uint uFlags, IntPtr dwItem1, IntPtr dwItem2);
}
'@
    Add-Type -TypeDefinition $code -ErrorAction SilentlyContinue
    [Shell32Win10Fix]::SHChangeNotify(0x08000000, 0x0000, [IntPtr]::Zero, [IntPtr]::Zero)
}

# ---------------------------------------------------------------------------
# Clear the UserChoice and OpenWithList that lock a file extension to one app.
# These are the registry entries that cause Windows 10 to suppress third-party
# context menu handlers when the locked app is a UWP/Store application.
# ---------------------------------------------------------------------------
function Clear-FileExtAssociation {
    param([string]$Ext)

    $base = "HKCU:\SOFTWARE\Microsoft\Windows\CurrentVersion\Explorer\FileExts\$Ext"

    $openWithList = "$base\OpenWithList"
    if (Test-Path $openWithList) {
        Remove-Item -Path $openWithList -Recurse -Force -ErrorAction SilentlyContinue
        Write-Host "    Cleared OpenWithList for $Ext" -ForegroundColor Green
    } else {
        Write-Host "    OpenWithList not present for $Ext" -ForegroundColor Gray
    }

    $userChoice = "$base\UserChoice"
    if (Test-Path $userChoice) {
        $lockedProgId = (Get-ItemProperty -Path $userChoice -ErrorAction SilentlyContinue).ProgId
        Remove-Item -Path $userChoice -Recurse -Force -ErrorAction SilentlyContinue
        if (Test-Path $userChoice) {
            Write-Warning "    Could not remove UserChoice for $Ext (ProgId was: $lockedProgId). Try running as SYSTEM or use regedit manually."
        } else {
            Write-Host "    Cleared UserChoice (was: $lockedProgId) for $Ext" -ForegroundColor Green
        }
    } else {
        Write-Host "    UserChoice not present for $Ext" -ForegroundColor Gray
    }
}

# ---------------------------------------------------------------------------
# Register under SystemFileAssociations.
# This key is consulted regardless of which app owns the UserChoice ProgId,
# making it the persistent solution that survives future file-open operations.
# ---------------------------------------------------------------------------
function Register-SystemFileAssociations {
    param([string]$Ext)

    foreach ($root in @('HKCU:', 'HKLM:')) {
        $regPath = "$root\Software\Classes\SystemFileAssociations\$Ext\ShellEx\ContextMenuHandlers\ModelShellExtension"
        try {
            if (-not (Test-Path $regPath)) { New-Item -Path $regPath -Force | Out-Null }
            Set-ItemProperty -Path $regPath -Name "(default)" -Value $MenuGUID -ErrorAction Stop
            Write-Host "    Registered SystemFileAssociations\$Ext under $root" -ForegroundColor Green
        } catch {
            Write-Host "    Skipped $root (no write access - expected without admin): $_" -ForegroundColor Yellow
        }
    }
}

# ---------------------------------------------------------------------------
# Main registration flow
# ---------------------------------------------------------------------------
function Register-Extension {
    if (-not (Test-Path $InstalledDllPath)) {
        Write-Error "ModelShellExtension.dll not found at:`n  $InstalledDllPath`nPlease install the NCrews GLTF Shell Extension first."
        exit 1
    }

    Write-Host "Re-registering shell extension DLL..." -ForegroundColor Yellow
    & "$env:WINDIR\System32\regsvr32.exe" /s $InstalledDllPath
    Write-Host "  DLL registered." -ForegroundColor Green

    foreach ($ext in $ContextMenuExtensions) {
        Write-Host ""
        Write-Host "Processing $ext..." -ForegroundColor Yellow
        Clear-FileExtAssociation -Ext $ext
        Register-SystemFileAssociations -Ext $ext
        if ($ThumbnailExtensions -contains $ext) {
            Register-ThumbnailAssociations -Ext $ext
        } else {
            Write-Host "    Thumbnail registration skipped for $ext (requires external reference files)." -ForegroundColor Yellow
        }
    }

    Write-Host ""
    Write-Host "Refreshing Windows Shell..." -ForegroundColor Yellow
    Invoke-ShellRefresh

    Write-Host ""
    Write-Host "Registration applied. Close and re-open File Explorer, then test right-click and thumbnails on supported model files." -ForegroundColor Green
}

# ---------------------------------------------------------------------------
# Unregister: remove only the SystemFileAssociations entries added by this script
# ---------------------------------------------------------------------------
function Unregister-Extension {
    Write-Host "Removing registration entries..." -ForegroundColor Yellow

    foreach ($ext in $ContextMenuExtensions) {
        foreach ($root in @('HKCU:', 'HKLM:')) {
            $regPath = "$root\Software\Classes\SystemFileAssociations\$ext\ShellEx\ContextMenuHandlers\ModelShellExtension"
            if (Test-Path $regPath) {
                Remove-Item -Path $regPath -Recurse -Force -ErrorAction SilentlyContinue
                Write-Host "  Removed: $regPath" -ForegroundColor Green
            }
            $thumbPath = "$root\Software\Classes\$ext\ShellEx\$ThumbProviderKey"
            if (Test-Path $thumbPath) {
                Remove-Item -Path $thumbPath -Recurse -Force -ErrorAction SilentlyContinue
                Write-Host "  Removed: $thumbPath" -ForegroundColor Green
            }
        }
    }

    Invoke-ShellRefresh
    Write-Host "Done." -ForegroundColor Green
}

# ---------------------------------------------------------------------------
# Test / Diagnose: report the current registry state without making changes
# ---------------------------------------------------------------------------
function Test-Extension {
    Write-Host ""
    Write-Host "Diagnosing registry state for context menu handlers..." -ForegroundColor Yellow

    foreach ($ext in $ContextMenuExtensions) {
        Write-Host ""
        Write-Host "$ext" -ForegroundColor Cyan

        $base = "HKCU:\SOFTWARE\Microsoft\Windows\CurrentVersion\Explorer\FileExts\$ext"

        # UserChoice
        $uc = "$base\UserChoice"
        if (Test-Path $uc) {
            $progId = (Get-ItemProperty -Path $uc -ErrorAction SilentlyContinue).ProgId
            $color  = if ($progId -like 'AppX*') { 'Red' } else { 'Yellow' }
            Write-Host "  UserChoice ProgId : $progId" -ForegroundColor $color
            if ($progId -like 'AppX*') {
                Write-Host "  WARNING: UWP/Store ProgId detected - this blocks third-party context menus on Windows 10." -ForegroundColor Red
            }
        } else {
            Write-Host "  UserChoice        : not set" -ForegroundColor Green
        }

        # OpenWithList
        $owl = "$base\OpenWithList"
        if (Test-Path $owl) {
            Write-Host "  OpenWithList      : present" -ForegroundColor Yellow
        } else {
            Write-Host "  OpenWithList      : not present" -ForegroundColor Green
        }

        # SystemFileAssociations
        foreach ($root in @('HKCU:', 'HKLM:')) {
            $regPath = "$root\Software\Classes\SystemFileAssociations\$ext\ShellEx\ContextMenuHandlers\ModelShellExtension"
            if (Test-Path $regPath) {
                $val = (Get-ItemProperty -Path $regPath -ErrorAction SilentlyContinue)."(default)"
                Write-Host "  SystemFileAssoc ($root): $val" -ForegroundColor Green
            } else {
                Write-Host "  SystemFileAssoc ($root): missing - run script without -Test to fix" -ForegroundColor Red
            }
        }

        # Thumbnail association
        if ($ThumbnailExtensions -contains $ext) {
            foreach ($root in @('HKCU:', 'HKLM:')) {
                $thumbPath = "$root\Software\Classes\$ext\ShellEx\$ThumbProviderKey"
                if (Test-Path $thumbPath) {
                    $val = (Get-ItemProperty -Path $thumbPath -ErrorAction SilentlyContinue)."(default)"
                    Write-Host "  ThumbnailAssoc ($root): $val" -ForegroundColor Green
                } else {
                    Write-Host "  ThumbnailAssoc ($root): missing - run script without -Test to fix" -ForegroundColor Red
                }
            }
        } else {
            Write-Host "  ThumbnailAssoc      : intentionally unsupported for $ext" -ForegroundColor Yellow
        }
    }

    Write-Host ""
    Write-Host "To apply the fix, run this script without any parameters (as Administrator)." -ForegroundColor Cyan
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

# ---------------------------------------------------------------------------
# Elevation: prompt for admin rights unless running -Test (read-only)
# ---------------------------------------------------------------------------
$isAdmin = ([Security.Principal.WindowsPrincipal] [Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole] "Administrator")

if (-not $isAdmin -and -not $Test) {
    Write-Host "Requesting elevation..." -ForegroundColor Yellow
    $psi = New-Object System.Diagnostics.ProcessStartInfo
    $psi.FileName = (Get-Process -Id $PID).Path
    $argList = @('-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', $PSCommandPath) + (
        $PSBoundParameters.GetEnumerator() | ForEach-Object {
            if ($_.Value -is [switch] -and $_.Value) { "-$($_.Key)" }
            elseif ($_.Value) { "-$($_.Key)"; "$($_.Value)" }
        }
    )
    $psi.Arguments = ($argList | ForEach-Object { if ($_ -match '\s') { "`"$_`"" } else { $_ } }) -join ' '
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

# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
Write-Host "NCrews GLTF Shell Extension - Windows 10 Registration" -ForegroundColor Cyan
Write-Host "=====================================================" -ForegroundColor Cyan

if ($Unregister) {
    Unregister-Extension
} elseif ($Test) {
    Test-Extension
} else {
    Register-Extension
}

Show-DiagnosticLogInfo

Write-Host ""
Write-Host "Press any key to exit..." -ForegroundColor Cyan
$null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")
