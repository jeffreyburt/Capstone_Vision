# Check the robot network from the controller laptop (docs/NETWORK.md section 7).
# No administrator rights needed. From the Capstone_Vision folder:
#   powershell -ExecutionPolicy Bypass -File network\check_laptop.ps1
#   powershell -ExecutionPolicy Bypass -File network\check_laptop.ps1 -User pi
#
# With -User, it also logs in over SSH (needs SSH keys, see network/README.md) and checks that
# each robot's chrony is synced to the master Pi. Prints PASS / WARN / FAIL; exit code = number of FAILs.
param([string]$User = "")

$MasterIp = "192.168.50.1"
$LaptopIp = "192.168.50.10"
$Pis = @(
    @{ Name = "master-pi"; Ip = "192.168.50.1";  Port = 5600; Stream = "birdseye" },
    @{ Name = "robot1-pi"; Ip = "192.168.50.11"; Port = 5601; Stream = "robot1" },
    @{ Name = "robot2-pi"; Ip = "192.168.50.12"; Port = 5602; Stream = "robot2" }
)
$script:Fails = 0
$script:Warns = 0

function Pass($msg) { Write-Host "  PASS  " -ForegroundColor Green -NoNewline; Write-Host $msg }
function Warn($msg) { Write-Host "  WARN  " -ForegroundColor Yellow -NoNewline; Write-Host $msg; $script:Warns++ }
function Fail($msg) { Write-Host "  FAIL  " -ForegroundColor Red -NoNewline; Write-Host $msg; $script:Fails++ }
function Section($msg) { Write-Host ""; Write-Host "== $msg ==" }

function Test-Tcp($ip, $port) {
    $c = New-Object System.Net.Sockets.TcpClient
    try { return $c.ConnectAsync($ip, $port).Wait(2000) } catch { return $false } finally { $c.Close() }
}

Write-Host "Laptop network check - $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')"

# ---- Laptop's own Ethernet settings ----
Section "Laptop Ethernet"
$addr = Get-NetIPAddress -IPAddress $LaptopIp -ErrorAction SilentlyContinue
if ($null -eq $addr) {
    Fail "no adapter has $LaptopIp. Run network\setup_laptop.ps1 as administrator."
} else {
    $alias = $addr.InterfaceAlias
    Pass "$LaptopIp is on '$alias'"
    if ($addr.PrefixLength -eq 24) { Pass "prefix /24" } else { Fail "prefix is /$($addr.PrefixLength), expected /24" }
    $gw = Get-NetRoute -InterfaceAlias $alias -DestinationPrefix "0.0.0.0/0" -ErrorAction SilentlyContinue
    if ($null -eq $gw) { Pass "no default gateway on '$alias' (internet stays on WiFi)" }
    else { Fail "'$alias' has a default gateway $($gw.NextHop). Remove it or internet may break." }
    $adapter = Get-NetAdapter -InterfaceAlias $alias
    if ($adapter.Status -eq "Up") { Pass "link up ($($adapter.LinkSpeed))" } else { Fail "link is $($adapter.Status) - cable plugged in and master Pi on?" }
    $prof = Get-NetConnectionProfile -InterfaceAlias $alias -ErrorAction SilentlyContinue
    if ($null -eq $prof) { Warn "no network profile yet (link down?)" }
    elseif ($prof.NetworkCategory -eq "Private") { Pass "network profile Private" }
    else { Warn "network profile is $($prof.NetworkCategory) (the Capstone firewall rules still work; see README to make it Private)" }
}
$rules = @(Get-NetFirewallRule -Group "Capstone robot network" -ErrorAction SilentlyContinue | Where-Object { $_.Enabled -eq "True" })
if ($rules.Count -ge 2) { Pass "firewall rules present (UDP 5005, ping)" } else { Fail "Capstone firewall rules missing. Run network\setup_laptop.ps1 as administrator." }
$inet = Get-NetRoute -DestinationPrefix "0.0.0.0/0" -ErrorAction SilentlyContinue | Where-Object { $_.NextHop -notlike "192.168.50.*" }
if ($inet) { Pass "internet route still via another adapter ($(@($inet)[0].InterfaceAlias))" } else { Warn "no internet route at all (laptop WiFi off?)" }

# ---- Each Pi ----
foreach ($pi in $Pis) {
    Section "$($pi.Name) ($($pi.Ip))"
    $ping = Test-Connection -ComputerName $pi.Ip -Count 3 -ErrorAction SilentlyContinue
    if ($ping) {
        $avg = [math]::Round((@($ping) | Measure-Object -Property ResponseTime -Average).Average, 1)
        Pass "ping ($avg ms average)"
    } else { Fail "ping - no answer" }
    if (Test-Tcp $pi.Ip 22) { Pass "SSH port 22 open" } else { Fail "SSH port 22 closed" }
    if (Test-Tcp $pi.Ip $pi.Port) { Pass "camera stream port $($pi.Port) open" }
    else { Warn "camera stream port $($pi.Port) closed (fine if 'python -m camstream.sender $($pi.Stream)' isn't running)" }

    if ($User -ne "") {
        $sshArgs = @("-o", "BatchMode=yes", "-o", "ConnectTimeout=5", "-o", "StrictHostKeyChecking=accept-new", "-o", "LogLevel=ERROR", "$User@$($pi.Ip)")
        $out = & ssh @sshArgs "hostname; chronyc -n tracking" 2>&1 | Out-String
        if ($LASTEXITCODE -ne 0) {
            Fail "ssh $User@$($pi.Ip) failed: $($out.Trim()) (password login can't be tested automatically; set up keys, see README)"
        } else {
            $hn = ($out -split "`n")[0].Trim()
            if ($hn -eq $pi.Name) { Pass "ssh login works (hostname $hn)" } else { Warn "ssh login works but hostname is '$hn'" }
            if ($pi.Name -ne "master-pi") {
                if ($out -match "Reference ID\s*:\s*\S+ \($([regex]::Escape($MasterIp))\)") { Pass "chrony synced to $MasterIp" }
                else { Fail "chrony not synced to $MasterIp (run 'chronyc -n sources' on the Pi)" }
                if ($out -match "System time\s*:\s*([0-9.]+) seconds") {
                    $off = [double]$Matches[1]
                    if ($off -lt 0.005) { Pass "clock offset $([math]::Round($off * 1000, 2)) ms (< 5 ms)" }
                    else { Warn "clock offset $([math]::Round($off * 1000, 2)) ms (target < 5 ms)" }
                }
            }
        }
    }
}

# ---- Master Pi time server ----
Section "Time server on master-pi (NTP, UDP 123)"
$st = & w32tm /stripchart /computer:$MasterIp /samples:1 /dataonly 2>&1 | Out-String
if ($st -match "([+-]\d+\.\d+)s") {
    Pass "master-pi answers NTP (laptop clock differs by $($Matches[1]) s; syncing the laptop is optional)"
} else {
    Fail "master-pi doesn't answer NTP: $($st.Trim())"
}
if ($User -eq "") { Write-Host "  (Add -User <pi username> to also check SSH login and chrony sync on the robots.)" }

Write-Host ""
if ($script:Fails -eq 0) { Write-Host "RESULT: ALL REQUIRED CHECKS PASSED ($($script:Warns) WARN)" -ForegroundColor Green }
else { Write-Host "RESULT: $($script:Fails) FAIL, $($script:Warns) WARN - see network/README.md -> Troubleshooting" -ForegroundColor Red }
exit $script:Fails
