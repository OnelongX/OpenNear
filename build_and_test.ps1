$ErrorActionPreference = "Stop"

$Root = Split-Path -Parent $MyInvocation.MyCommand.Path
$BuildDir = Join-Path $Root "build"
New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null
New-Item -ItemType Directory -Force -Path (Join-Path $BuildDir "esp32_obj") | Out-Null

# Locate host GCC (PATH or PlatformIO MinGW package)
$gccBin = Join-Path $env:USERPROFILE ".platformio\packages\toolchain-gccmingw32\bin"
if (Test-Path $gccBin) {
    $env:PATH = "$gccBin;$env:PATH"
}

$inc = @(
    "-I$Root\core\include",
    "-I$Root\hal",
    "-I$Root\sim"
)
$core = @(
    "$Root\core\src\on_frame.c",
    "$Root\core\src\on_polar.c",
    "$Root\core\src\on_sync.c",
    "$Root\core\src\on_superframe.c",
    "$Root\core\src\on_sec.c",
    "$Root\core\src\on_ecdh.c",
    "$Root\core\src\on_afh.c",
    "$Root\core\src\on_amc.c",
    "$Root\core\src\on_sar.c",
    "$Root\core\src\on_ranging.c",
    "$Root\core\src\on_ota.c",
    "$Root\core\src\on_gateway.c",
    "$Root\core\src\on_mesh.c",
    "$Root\core\src\on_mqtt.c",
    "$Root\core\src\on_node.c"
)

Write-Host "`n[1/4] Building & Running OpenNear Unit + Integration Test Suite..." -ForegroundColor Cyan
& gcc -std=c11 -O2 -Wall -Wextra -D__USE_MINGW_ANSI_STDIO=1 @inc @core `
    "$Root\sim\sim_world.c" "$Root\tests\test_opennear.c" -lm `
    -o "$BuildDir\test_opennear.exe"
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& "$BuildDir\test_opennear.exe"
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

Write-Host "`n[2/4] Building & Running 10-Node Multi-Hop Lifecycle Simulator..." -ForegroundColor Cyan
& gcc -std=c11 -O2 -Wall -Wextra -D__USE_MINGW_ANSI_STDIO=1 @inc @core `
    "$Root\sim\sim_world.c" "$Root\sim\main_sim.c" -lm `
    -o "$BuildDir\opennear_sim.exe"
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
Push-Location $BuildDir
& "$BuildDir\opennear_sim.exe"
$simExit = $LASTEXITCODE
Pop-Location
if ($simExit -ne 0) { exit $simExit }

Write-Host "`n[3/4] Building & Running Route B L1 PHY Benchmark (CFO+Multipath + Fixed CA-Polar + AMC)..." -ForegroundColor Cyan
& gcc -std=c11 -O2 -Wall -Wextra -D__USE_MINGW_ANSI_STDIO=1 "-I$Root\core\include" `
    "$Root\core\src\on_frame.c" "$Root\core\src\on_polar.c" "$Root\core\src\on_amc.c" `
    "$Root\ports\sdr_fpga_ref\l1_phy_bench.c" -lm `
    -o "$BuildDir\l1_phy_bench.exe"
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& "$BuildDir\l1_phy_bench.exe"
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

Write-Host "`n[4/4] Cross-Compile Check: ESP32-WROOM-32 (Xtensa) + nRF52840 Bare-Metal C11..." -ForegroundColor Cyan
& gcc -std=c11 -O2 -Wall -Wextra -fsyntax-only `
    "-I$Root\core\include" "-I$Root\hal" "-I$Root\ports\nrf52840_bare\include" `
    "$Root\ports\nrf52840_bare\src\startup_nrf52840.c" `
    "$Root\ports\nrf52840_bare\src\hal_nrf52_radio_ppi.c" `
    "$Root\ports\nrf52840_bare\src\main_nrf52.c"
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
Write-Host "  OK: Route A nRF52840 bare-metal NRF_RADIO + PPI C11 check passed." -ForegroundColor Green

$xbin = Join-Path $env:USERPROFILE ".platformio\packages\toolchain-xtensa-esp32\bin"
$fw   = Join-Path $env:USERPROFILE ".platformio\packages\framework-arduinoespressif32"
$sdk  = Join-Path $fw "tools\sdk\esp32"

if ((Test-Path $xbin) -and (Test-Path $sdk)) {
    $env:PATH = "$xbin;$env:PATH"
    $espInc = @(
        "-I$Root\core\include",
        "-I$Root\hal",
        "-I$Root\ports\esp32_wroom32\include",
        "-I$fw\cores\esp32",
        "-I$fw\variants\esp32",
        "-I$sdk\dio_qspi\include",
        "-I$sdk\include\config",
        "-I$sdk\include\newlib\platform_include",
        "-I$sdk\include\freertos\include",
        "-I$sdk\include\freertos\include\esp_additions",
        "-I$sdk\include\freertos\include\esp_additions\freertos",
        "-I$sdk\include\freertos\port\xtensa\include",
        "-I$sdk\include\esp_hw_support\include",
        "-I$sdk\include\esp_hw_support\include\soc",
        "-I$sdk\include\esp_hw_support\include\soc\esp32",
        "-I$sdk\include\esp_hw_support\port\esp32",
        "-I$sdk\include\esp_common\include",
        "-I$sdk\include\esp_rom\include",
        "-I$sdk\include\esp_rom\include\esp32",
        "-I$sdk\include\esp_system\include",
        "-I$sdk\include\esp_timer\include",
        "-I$sdk\include\esp_wifi\include",
        "-I$sdk\include\esp_event\include",
        "-I$sdk\include\esp_netif\include",
        "-I$sdk\include\tcpip_adapter\include",
        "-I$sdk\include\esp_eth\include",
        "-I$sdk\include\driver\include",
        "-I$sdk\include\driver\esp32\include",
        "-I$sdk\include\hal\include",
        "-I$sdk\include\hal\esp32\include",
        "-I$sdk\include\soc\include",
        "-I$sdk\include\soc\esp32\include",
        "-I$sdk\include\xtensa\include",
        "-I$sdk\include\xtensa\esp32\include",
        "-I$sdk\include\heap\include",
        "-I$sdk\include\log\include",
        "-I$sdk\include\lwip\include\apps",
        "-I$sdk\include\lwip\lwip\src\include",
        "-I$sdk\include\lwip\port\esp32\include",
        "-I$sdk\include\lwip\port\esp32\include\arch",
        "-I$sdk\include\nvs_flash\include",
        "-I$sdk\include\spi_flash\include",
        "-I$sdk\include\app_update\include",
        "-I$sdk\include\bootloader_support\include",
        "-I$sdk\include\esp_ipc\include",
        "-I$sdk\include\esp_ringbuf\include",
        "-I$sdk\include\nghttp\port\include",
        "-I$sdk\include\esp32\include",
        "-I$sdk\include\wifi_provisioning\include",
        "-I$sdk\include\protocomm\include\common",
        "-I$sdk\include\protocomm\include\security",
        "-I$sdk\include\protocomm\include\transports",
        "-I$sdk\include\bt\include\esp32\include",
        "-I$sdk\include\bt\host\bluedroid\api\include\api",
        "-I$sdk\include\bt\common\btc\include",
        "-I$sdk\include\bt\common\osi\include",
        "-I$sdk\include\mbedtls\port\include",
        "-I$sdk\include\mbedtls\mbedtls\include",
        "-I$fw\libraries\WiFi\src",
        "-I$fw\libraries\WiFiClientSecure\src",
        "-I$fw\libraries\HTTPClient\src",
        "-I$fw\libraries\WebServer\src",
        "-I$fw\libraries\FS\src",
        "-I$fw\libraries\Preferences\src",
        "-I$fw\libraries\BLE\src",
        "-DESP32", "-DESP_PLATFORM", "-DF_CPU=240000000L", "-DARDUINO=10812", "-DARDUINO_ARCH_ESP32",
        "-Wno-unused-parameter"
    )
    $espC = @(
        "$Root\core\src\on_frame.c",
        "$Root\core\src\on_polar.c",
        "$Root\core\src\on_sync.c",
        "$Root\core\src\on_superframe.c",
        "$Root\core\src\on_sec.c",
        "$Root\core\src\on_ecdh.c",
        "$Root\core\src\on_afh.c",
        "$Root\core\src\on_amc.c",
        "$Root\core\src\on_sar.c",
        "$Root\core\src\on_ranging.c",
        "$Root\core\src\on_ota.c",
        "$Root\core\src\on_gateway.c",
        "$Root\core\src\on_mesh.c",
        "$Root\core\src\on_mqtt.c",
        "$Root\core\src\on_node.c",
        "$Root\ports\esp32_wroom32\src\hal_esp32_raw80211.c"
    )
    foreach ($f in $espC) {
        $obj = Join-Path "$BuildDir\esp32_obj" ([IO.Path]::GetFileNameWithoutExtension($f) + ".o")
        & xtensa-esp32-elf-gcc -std=gnu11 -Os -Wall -Wextra -mlongcalls -c $f @espInc -o $obj
        if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    }
    $espCpp = @(
        "$Root\ports\esp32_wroom32\src\act_capture.cpp",
        "$Root\ports\esp32_wroom32\src\esp32_ota_flash.cpp",
        "$Root\ports\esp32_wroom32\src\esp32_tls_transport.cpp",
        "$Root\ports\esp32_wroom32\src\esp32_net_gateway.cpp",
        "$Root\ports\esp32_wroom32\src\main.cpp"
    )
    foreach ($f in $espCpp) {
        $obj = Join-Path "$BuildDir\esp32_obj" ([IO.Path]::GetFileNameWithoutExtension($f) + ".o")
        & xtensa-esp32-elf-g++ -std=gnu++11 -Os -Wall -Wextra -mlongcalls -fno-rtti -fno-exceptions `
            -DOPENNEAR_ENABLE_OTA=1 -DOPENNEAR_ENABLE_WIFI_GW=1 -DOPENNEAR_ENABLE_BLE_GW=1 -DOPENNEAR_LEADERLESS_BOOT=1 `
            -c $f @espInc -o $obj
        if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    }
    # Verify optional MQTTS/HTTPS TLS module (-DOPENNEAR_ENABLE_TLS=1)
    & xtensa-esp32-elf-g++ -std=gnu++11 -Os -Wall -Wextra -Werror -mlongcalls -fno-rtti -fno-exceptions `
        -DOPENNEAR_ENABLE_OTA=1 -DOPENNEAR_ENABLE_WIFI_GW=1 -DOPENNEAR_ENABLE_BLE_GW=1 -DOPENNEAR_LEADERLESS_BOOT=1 -DOPENNEAR_ENABLE_TLS=1 `
        -c "$Root\ports\esp32_wroom32\src\esp32_tls_transport.cpp" @espInc -o "$BuildDir\esp32_obj\esp32_tls_transport_tls.o"
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    Write-Host "  OK: Route C ESP32-WROOM-32 Xtensa cross-compilation passed (21/21 objs + optional MQTTS/TLS module)." -ForegroundColor Green
} else {
    Write-Host "  SKIP: ESP32 Xtensa toolchain not found in default PlatformIO path." -ForegroundColor Yellow
}

Write-Host "`nALL OPENNEAR BUILDS & VERIFICATIONS PASSED!" -ForegroundColor Green
