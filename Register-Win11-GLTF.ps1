# Register GLTF Shell Extension for Windows 11 Compatibility
# Run as Administrator

param(
    [switch]$Unregister,
    [switch]$Test,
    [switch]$ClearCache
)

$ErrorActionPreference = "Continue"

# Paths
$ProjectRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$DllPath = Join-Path $ProjectRoot "build\Release\ModelShellExtension\ModelShellExtension.dll"
$BackupDllPath = Join-Path $ProjectRoot "build\Release\ModelShellExtension.dll"

# Find the actual DLL
if (Test-Path $DllPath) {
    $ActualDllPath = $DllPath
} elseif (Test-Path $BackupDllPath) {
    $ActualDllPath = $BackupDllPath
} else {
    Write-Error "ModelShellExtension.dll not found. Please build the project first."
    exit 1
}

function Clear-ThumbnailCache {
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

function Register-Extension {
    Write-Host "Registering GLTF Shell Extension..." -ForegroundColor Yellow
    
    # Register the DLL
    $result = & "regsvr32.exe" "/s" $ActualDllPath
    if ($LASTEXITCODE -eq 0) {
        Write-Host "DLL registered successfully." -ForegroundColor Green
    } else {
        Write-Error "Failed to register DLL. Error code: $LASTEXITCODE"
        return
    }
    
    # Additional Windows 11 registry entries
    Write-Host "Adding Windows 11 specific registry entries..." -ForegroundColor Yellow
    
    # GUID for ModelShellExtension (these should match your actual GUIDs)
    $MenuGUID = "{2E7F05D1-89D7-4028-8C63-5606D35ECA69}"  # Replace with actual GUID
    $ThumbnailGUID = "{1DD4D323-8CA7-4F68-9369-730DF074A0F0}"  # Replace with actual GUID
    
    try {
        # Ensure proper thumbnail provider registration
        $regPath = "HKCU:\Software\Classes\.glb\ShellEx\{E357FCCD-A995-4576-B01F-234630154E96}"
        if (-not (Test-Path $regPath)) {
            New-Item -Path $regPath -Force | Out-Null
        }
        Set-ItemProperty -Path $regPath -Name "(default)" -Value $ThumbnailGUID
        
        # Add context menu handler for GLB files
        $regPath = "HKCU:\Software\Classes\.glb\ShellEx\ContextMenuHandlers\ModelShellExtension"
        if (-not (Test-Path $regPath)) {
            New-Item -Path $regPath -Force | Out-Null
        }
        Set-ItemProperty -Path $regPath -Name "(default)" -Value $MenuGUID
        
        # Add for GLTF files as well
        $regPath = "HKCU:\Software\Classes\.gltf\ShellEx\ContextMenuHandlers\ModelShellExtension"
        if (-not (Test-Path $regPath)) {
            New-Item -Path $regPath -Force | Out-Null
        }
        Set-ItemProperty -Path $regPath -Name "(default)" -Value $MenuGUID
        
        Write-Host "Registry entries added successfully." -ForegroundColor Green
        
    } catch {
        Write-Error "Failed to add registry entries: $_"
    }
    
    # Notify shell of changes
    Write-Host "Notifying Windows Shell of changes..." -ForegroundColor Yellow
    
    Add-Type -TypeDefinition @"
        using System;
        using System.Runtime.InteropServices;
        public class Shell32 {
            [DllImport("shell32.dll")]
            public static extern void SHChangeNotify(uint wEventId, uint uFlags, IntPtr dwItem1, IntPtr dwItem2);
        }
"@
    
    [Shell32]::SHChangeNotify(0x08000000, 0x0000, [IntPtr]::Zero, [IntPtr]::Zero)  # SHCNE_ASSOCCHANGED
}

function Unregister-Extension {
    Write-Host "Unregistering GLTF Shell Extension..." -ForegroundColor Yellow
    
    # Unregister the DLL
    $result = & "regsvr32.exe" "/u" "/s" $ActualDllPath
    if ($LASTEXITCODE -eq 0) {
        Write-Host "DLL unregistered successfully." -ForegroundColor Green
    } else {
        Write-Warning "Failed to unregister DLL. Error code: $LASTEXITCODE"
    }
    
    # Remove registry entries
    try {
        Remove-Item -Path "HKCU:\Software\Classes\.glb\ShellEx\ContextMenuHandlers\ModelShellExtension" -Recurse -Force -ErrorAction SilentlyContinue
        Remove-Item -Path "HKCU:\Software\Classes\.gltf\ShellEx\ContextMenuHandlers\ModelShellExtension" -Recurse -Force -ErrorAction SilentlyContinue
        Write-Host "Registry entries removed." -ForegroundColor Green
    } catch {
        Write-Warning "Some registry entries could not be removed: $_"
    }
}

function Test-Extension {
    Write-Host "Testing GLTF Shell Extension..." -ForegroundColor Yellow
    
    # Check if DLL exists and is registered
    if (-not (Test-Path $ActualDllPath)) {
        Write-Error "DLL not found at $ActualDllPath"
        return
    }
    
    Write-Host "DLL found: $ActualDllPath" -ForegroundColor Green
    
    # Check registry entries
    $regPaths = @(
        "HKCU:\Software\Classes\.glb\ShellEx\{E357FCCD-A995-4576-B01F-234630154E96}",
        "HKCU:\Software\Classes\.glb\ShellEx\ContextMenuHandlers\ModelShellExtension",
        "HKCU:\Software\Classes\.gltf\ShellEx\ContextMenuHandlers\ModelShellExtension"
    )
    
    foreach ($regPath in $regPaths) {
        if (Test-Path $regPath) {
            $value = (Get-ItemProperty -Path $regPath -Name "(default)" -ErrorAction SilentlyContinue)."(default)"
            Write-Host "✓ Registry entry found: $regPath = $value" -ForegroundColor Green
        } else {
            Write-Host "✗ Missing registry entry: $regPath" -ForegroundColor Red
        }
    }
    
    Write-Host "`nTo test thumbnails:" -ForegroundColor Cyan
    Write-Host "1. Create or copy a .glb file to your desktop" -ForegroundColor Cyan
    Write-Host "2. Switch to thumbnail view in File Explorer" -ForegroundColor Cyan
    Write-Host "3. Check if custom thumbnail appears" -ForegroundColor Cyan
    Write-Host "4. Right-click the file to see context menu" -ForegroundColor Cyan
}

# Main execution
if (-not ([Security.Principal.WindowsPrincipal] [Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole] "Administrator")) {
    Write-Error "This script must be run as Administrator for proper registration."
    exit 1
}

Write-Host "GLTF Shell Extension Manager for Windows 11" -ForegroundColor Cyan
Write-Host "============================================" -ForegroundColor Cyan

if ($ClearCache) {
    Clear-ThumbnailCache
}

if ($Unregister) {
    Unregister-Extension
} elseif ($Test) {
    Test-Extension
} else {
    Register-Extension
    Write-Host "`nRecommended next steps:" -ForegroundColor Yellow
    Write-Host "1. Run: .\Register-Win11-GLTF.ps1 -ClearCache" -ForegroundColor Yellow
    Write-Host "2. Run: .\Register-Win11-GLTF.ps1 -Test" -ForegroundColor Yellow
    Write-Host "3. Test with actual .glb/.gltf files" -ForegroundColor Yellow
}

Write-Host "`nDone!" -ForegroundColor Green