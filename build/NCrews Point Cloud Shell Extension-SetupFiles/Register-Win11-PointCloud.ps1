# Register Point Cloud Shell Extension for Windows 11 Compatibility
# Run as Administrator

param(
    [switch]$Unregister,
    [switch]$Test,
    [switch]$ClearCache,
    [switch]$ResetOpenWith
)

$ErrorActionPreference = "Continue"

# Paths
$InstalledDllPath = "C:\Program Files\NCrews Software\NCrews Point Cloud Shell Extension\CloudShellExtension.dll"

# Verify the installed DLL exists
if (-not (Test-Path $InstalledDllPath)) {
    Write-Error "CloudShellExtension.dll not found at $InstalledDllPath`nPlease ensure the NCrews Point Cloud Shell Extension is installed."
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
    
    # GUID for CloudShellExtension (from inc/CloudShellExtension/CloudMenuGUID.h)
    $MenuGUID = "{C2E79339-590A-4B13-AFCF-3A31FB39C7A8}"
    
    try {
        # Add context menu handler for LAS files
        foreach ($root in @('HKCU:','HKLM:')) {
            # Register on the extension key
            $regPath = "$root\Software\Classes\.las\ShellEx\ContextMenuHandlers\CloudShellExtension"
            if (-not (Test-Path $regPath)) { New-Item -Path $regPath -Force | Out-Null }
            Set-ItemProperty -Path $regPath -Name "(default)" -Value $MenuGUID
            
            $regPath = "$root\Software\Classes\.laz\ShellEx\ContextMenuHandlers\CloudShellExtension"
            if (-not (Test-Path $regPath)) { New-Item -Path $regPath -Force | Out-Null }
            Set-ItemProperty -Path $regPath -Name "(default)" -Value $MenuGUID

            # Also register under SystemFileAssociations to ensure per-file menu on Win10/11
            $regPath = "$root\Software\Classes\SystemFileAssociations\.las\ShellEx\ContextMenuHandlers\CloudShellExtension"
            if (-not (Test-Path $regPath)) { New-Item -Path $regPath -Force | Out-Null }
            Set-ItemProperty -Path $regPath -Name "(default)" -Value $MenuGUID

            $regPath = "$root\Software\Classes\SystemFileAssociations\.laz\ShellEx\ContextMenuHandlers\CloudShellExtension"
            if (-not (Test-Path $regPath)) { New-Item -Path $regPath -Force | Out-Null }
            Set-ItemProperty -Path $regPath -Name "(default)" -Value $MenuGUID
            
            # Directory handler
            $regPath = "$root\Software\Classes\Directory\ShellEx\ContextMenuHandlers\CloudShellExtension"
            if (-not (Test-Path $regPath)) { New-Item -Path $regPath -Force | Out-Null }
            Set-ItemProperty -Path $regPath -Name "(default)" -Value $MenuGUID
        }

        # Also bind to current ProgID (if any) so 3rd-party associations still get our menu
        Add-HandlerToProgId -Ext '.las' -HandlerName 'CloudShellExtension' -Guid $MenuGUID
        Add-HandlerToProgId -Ext '.laz' -HandlerName 'CloudShellExtension' -Guid $MenuGUID
        
        # Add to approved shell extensions (system-wide)
        $regPath = "HKLM:\SOFTWARE\Microsoft\Windows\CurrentVersion\Shell Extensions\Approved"
        if (-not (Test-Path $regPath)) { New-Item -Path $regPath -Force | Out-Null }
        New-ItemProperty -Path $regPath -Name $MenuGUID -PropertyType String -Value "Point Cloud Shell Extension" -Force | Out-Null
        
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
    Write-Host "Unregistering Point Cloud Shell Extension..." -ForegroundColor Yellow
    
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
            Remove-Item -Path "$root\Software\Classes\.las\ShellEx\ContextMenuHandlers\CloudShellExtension" -Recurse -Force -ErrorAction SilentlyContinue
            Remove-Item -Path "$root\Software\Classes\.laz\ShellEx\ContextMenuHandlers\CloudShellExtension" -Recurse -Force -ErrorAction SilentlyContinue
            Remove-Item -Path "$root\Software\Classes\SystemFileAssociations\.las\ShellEx\ContextMenuHandlers\CloudShellExtension" -Recurse -Force -ErrorAction SilentlyContinue
            Remove-Item -Path "$root\Software\Classes\SystemFileAssociations\.laz\ShellEx\ContextMenuHandlers\CloudShellExtension" -Recurse -Force -ErrorAction SilentlyContinue
            Remove-Item -Path "$root\Software\Classes\Directory\ShellEx\ContextMenuHandlers\CloudShellExtension" -Recurse -Force -ErrorAction SilentlyContinue
            # Remove from bound ProgIDs
            foreach($ext in '.las','.laz'){
                $hkcrDefault = (Get-ItemProperty -Path ("Registry::HKEY_CLASSES_ROOT\\$ext") -ErrorAction SilentlyContinue)."(default)"
                $userChoice = (Get-ItemProperty -Path ("HKCU:\\Software\\Microsoft\\Windows\\CurrentVersion\\Explorer\\FileExts\\$ext\\UserChoice") -ErrorAction SilentlyContinue).ProgId
                $progId = if ($userChoice) { $userChoice } else { $hkcrDefault }
                if ($progId) {
                    Remove-Item -Path "$root\Software\Classes\$progId\ShellEx\ContextMenuHandlers\CloudShellExtension" -Recurse -Force -ErrorAction SilentlyContinue
                }
            }
        }
        Write-Host "Registry entries removed." -ForegroundColor Green
    } catch {
        Write-Warning "Some registry entries could not be removed: $_"
    }
}

function Test-Extension {
    Write-Host "Testing Point Cloud Shell Extension..." -ForegroundColor Yellow
    
    Write-Host "DLL location: $ActualDllPath" -ForegroundColor Green
    
    # Check registry entries
    $regPaths = @(
        "HKCU:\Software\Classes\.las\ShellEx\ContextMenuHandlers\CloudShellExtension",
        "HKCU:\Software\Classes\.laz\ShellEx\ContextMenuHandlers\CloudShellExtension",
        "HKCU:\Software\Classes\SystemFileAssociations\.las\ShellEx\ContextMenuHandlers\CloudShellExtension",
        "HKCU:\Software\Classes\SystemFileAssociations\.laz\ShellEx\ContextMenuHandlers\CloudShellExtension",
        "HKCU:\Software\Classes\Directory\ShellEx\ContextMenuHandlers\CloudShellExtension",
        "HKLM:\Software\Classes\.las\ShellEx\ContextMenuHandlers\CloudShellExtension",
        "HKLM:\Software\Classes\.laz\ShellEx\ContextMenuHandlers\CloudShellExtension",
        "HKLM:\Software\Classes\SystemFileAssociations\.las\ShellEx\ContextMenuHandlers\CloudShellExtension",
        "HKLM:\Software\Classes\SystemFileAssociations\.laz\ShellEx\ContextMenuHandlers\CloudShellExtension",
        "HKLM:\Software\Classes\Directory\ShellEx\ContextMenuHandlers\CloudShellExtension"
    )
    
    foreach ($regPath in $regPaths) {
        if (Test-Path $regPath) {
            $value = (Get-ItemProperty -Path $regPath -Name "(default)" -ErrorAction SilentlyContinue)."(default)"
            Write-Host "Registry entry found: $regPath = $value" -ForegroundColor Green
        } else {
            Write-Host "Missing registry entry: $regPath" -ForegroundColor Red
        }
    }
    
    Write-Host "`nTo test point cloud processing:" -ForegroundColor Cyan
    Write-Host "1. Create or copy a .las or .laz file to your desktop" -ForegroundColor Cyan
    Write-Host "2. Right-click the file to see context menu options" -ForegroundColor Cyan
    Write-Host "3. Try 'Generate Point Cloud Image' menu item" -ForegroundColor Cyan
    Write-Host "4. Right-click a directory with point cloud files" -ForegroundColor Cyan
    Write-Host "5. Try batch processing options" -ForegroundColor Cyan
}

function Remove-TestHelpers {
    # Function removed - no longer needed for customer deployment
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

Write-Host "Point Cloud Shell Extension Manager for Windows 11" -ForegroundColor Cyan
Write-Host "====================================================" -ForegroundColor Cyan

if ($Unregister) {
    Unregister-Extension
} elseif ($Test) {
    Test-Extension
    if ($ResetOpenWith) { Reset-OpenWithList -Extensions @('.las','.laz') }
} elseif ($ResetOpenWith) {
    Reset-OpenWithList -Extensions @('.las','.laz')
} else {
    Register-Extension
    # Automatically clear cache after registration
    Clear-ThumbnailCache
    Write-Host "`nRecommended next steps:" -ForegroundColor Yellow
    Write-Host "1. Refresh File Explorer or open a new window" -ForegroundColor Yellow
    Write-Host "2. Run: .\Register-Win11-PointCloud.ps1 -Test" -ForegroundColor Yellow
    Write-Host "3. Test with actual .las/.laz files" -ForegroundColor Yellow
}

if ($ClearCache) {
    Clear-ThumbnailCache
}

Write-Host "`nDone!" -ForegroundColor Green
Write-Host "`nPress any key to exit..." -ForegroundColor Cyan
$null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")
