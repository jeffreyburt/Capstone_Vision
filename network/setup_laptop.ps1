# Controller laptop network setup (docs/NETWORK.md): static 192.168.50.10/24 on Ethernet with
# no gateway (internet stays on the laptop's WiFi), Private network profile, firewall rules.
#
# Run in an ADMINISTRATOR PowerShell, from the Capstone_Vision folder:
#   powershell -ExecutionPolicy Bypass -File network\setup_laptop.ps1
#   powershell -ExecutionPolicy Bypass -File network\setup_laptop.ps1 -InterfaceAlias "Ethernet 2"
#
# Safe to re-run. Undo: see network/README.md -> "Undo on the laptop".
param([string]$InterfaceAlias = "")

$ErrorActionPreference = "Stop"
$LaptopIp = "192.168.50.10"
$Subnet = "192.168.50.0/24"
$Group = "Capstone robot network"

$admin = ([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole(
    [Security.Principal.WindowsBuiltInRole]::Administrator)
if (-not $admin) { Write-Host "Run this in an Administrator PowerShell (right-click PowerShell -> Run as administrator)." -ForegroundColor Red; exit 1 }

# ---- Pick the Ethernet adapter ----
if ($InterfaceAlias -eq "") {
    $eth = @(Get-NetAdapter -Physical | Where-Object { $_.MediaType -eq "802.3" })
    if ($eth.Count -ne 1) {
        Write-Host "Found $($eth.Count) wired adapters. Pick one with -InterfaceAlias `"<Name>`":" -ForegroundColor Yellow
        Get-NetAdapter | Format-Table Name, InterfaceDescription, Status, MediaType
        exit 1
    }
    $InterfaceAlias = $eth[0].Name
}
Write-Host "== Using adapter '$InterfaceAlias' =="

# ---- Static IP, no gateway, no DNS ----
Set-NetIPInterface -InterfaceAlias $InterfaceAlias -AddressFamily IPv4 -Dhcp Disabled
Get-NetIPAddress -InterfaceAlias $InterfaceAlias -AddressFamily IPv4 -ErrorAction SilentlyContinue |
    Where-Object { $_.IPAddress -ne $LaptopIp } | Remove-NetIPAddress -Confirm:$false
Get-NetRoute -InterfaceAlias $InterfaceAlias -DestinationPrefix "0.0.0.0/0" -ErrorAction SilentlyContinue |
    Remove-NetRoute -Confirm:$false
if (-not (Get-NetIPAddress -InterfaceAlias $InterfaceAlias -IPAddress $LaptopIp -ErrorAction SilentlyContinue)) {
    New-NetIPAddress -InterfaceAlias $InterfaceAlias -IPAddress $LaptopIp -PrefixLength 24 | Out-Null
}
Set-DnsClientServerAddress -InterfaceAlias $InterfaceAlias -ResetServerAddresses
Write-Host "IPv4 $LaptopIp/24, no gateway, no DNS on '$InterfaceAlias'"

# ---- Firewall: allow vision output (UDP 5005) and ping from the robot network ----
# Scoped to the robot subnet + this PC, and for every profile, so it works even if Windows
# classifies the gateway-less Ethernet network as Public.
Get-NetFirewallRule -Group $Group -ErrorAction SilentlyContinue | Remove-NetFirewallRule
New-NetFirewallRule -Group $Group -DisplayName "Capstone: vision output UDP 5005" -Direction Inbound `
    -Protocol UDP -LocalPort 5005 -RemoteAddress $Subnet, "127.0.0.1" -Action Allow -Profile Any | Out-Null
New-NetFirewallRule -Group $Group -DisplayName "Capstone: ping from robot network" -Direction Inbound `
    -Protocol ICMPv4 -IcmpType 8 -RemoteAddress $Subnet -Action Allow -Profile Any | Out-Null
Write-Host "Firewall rules added (group '$Group')"

# ---- Network profile: Private ----
try {
    Set-NetConnectionProfile -InterfaceAlias $InterfaceAlias -NetworkCategory Private
    Write-Host "Network profile on '$InterfaceAlias' set to Private"
} catch {
    Write-Host "Couldn't set the network profile (is the cable to the master Pi plugged in and the Pi on?)." -ForegroundColor Yellow
    Write-Host "Plug it in and run this script again." -ForegroundColor Yellow
}
Write-Host ""
Write-Host "Note: a network without a gateway shows as 'Unidentified network', and Windows may reset it to"
Write-Host "Public after a reboot. The firewall rules above work either way. To make it stick as Private:"
Write-Host "  secpol.msc -> Network List Manager Policies -> Unidentified Networks -> Location type: Private"
Write-Host ""
Write-Host "Done. Check with:  powershell -ExecutionPolicy Bypass -File network\check_laptop.ps1" -ForegroundColor Green
