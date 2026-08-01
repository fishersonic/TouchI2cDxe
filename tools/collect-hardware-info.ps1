# collect-hardware-info.ps1 — Windows-side hardware intake for a new device
#
# Counterpart of collect-hardware-info.sh for machines that only have Windows
# installed. Run in an elevated PowerShell on the target device:
#
#   powershell -ExecutionPolicy Bypass -File .\collect-hardware-info.ps1
#
# Produces touch-hwinfo-<product>.zip next to the script, containing:
#   smbios.txt        exact DMI product/board strings (profile gate values)
#   acpi\*.aml        every ACPI table (DSDT + all SSDTs) from the registry
#   pnp-i2c.txt       I2C controllers and HID-over-I2C devices, with status
#   pnp-resources.txt MMIO ranges allocated to the I2C controllers
#
# The .aml tables decompile with `iasl -d` (or `acpica-tools`); the values a
# TouchI2cDxe profile needs are the controller _CRS memory base, the panel's
# I2cSerialBusV2 slave address, and the HID descriptor register from
# _DSM(3CDFF6F7-4267-4555-AD05-B30A3D8938DE, func 1).

$ErrorActionPreference = 'Continue'

$product = (Get-CimInstance Win32_ComputerSystemProduct).Name -replace '[^\w.-]', '_'
$outDir  = Join-Path $PSScriptRoot "touch-hwinfo-$product"
New-Item -ItemType Directory -Force -Path $outDir, (Join-Path $outDir 'acpi') | Out-Null

# --- SMBIOS identity (what the driver's DMI gate matches against) -----------
$csp   = Get-CimInstance Win32_ComputerSystemProduct
$board = Get-CimInstance Win32_BaseBoard
$bios  = Get-CimInstance Win32_BIOS
@(
  "SystemManufacturer : $($csp.Vendor)"
  "SystemProductName  : $($csp.Name)"          # SMBIOS Type 1 product
  "SystemVersion      : $($csp.Version)"
  "BaseBoardMfr       : $($board.Manufacturer)"
  "BaseBoardProduct   : $($board.Product)"     # SMBIOS Type 2 product
  "BIOSVersion        : $($bios.SMBIOSBIOSVersion)"
) | Set-Content (Join-Path $outDir 'smbios.txt')

# --- ACPI tables ------------------------------------------------------------
# HKLM\HARDWARE\ACPI holds every table the firmware published, one leaf key
# per table with the raw bytes in a binary value named "00000000". This gets
# all SSDTs individually, which GetSystemFirmwareTable cannot.
Get-ChildItem 'HKLM:\HARDWARE\ACPI' | ForEach-Object {
  $tbl = $_.PSChildName
  Get-ChildItem $_.PSPath -Recurse | ForEach-Object {
    $val = (Get-ItemProperty -Path $_.PSPath -Name '00000000' -ErrorAction SilentlyContinue).'00000000'
    if ($val) {
      [IO.File]::WriteAllBytes((Join-Path $outDir "acpi\$tbl.aml"), [byte[]]$val)
    }
  }
}

# --- I2C controllers and HID-over-I2C devices -------------------------------
$i2cLike = Get-PnpDevice -PresentOnly:$false | Where-Object {
  $_.InstanceId -match '^ACPI\\(INT[0-9A-F]{4}|INTC[0-9A-F]{4}|AMDI0010|ELAN|NVTK|GDIX|GXTP|FTS|SYNA|WCOM|ATML|PNP0C50)' -or
  $_.Class -eq 'HIDClass' -and $_.InstanceId -match 'I2C|ELAN'
}
$i2cLike | Sort-Object InstanceId |
  Format-List InstanceId, FriendlyName, Class, Status, Problem |
  Out-String -Width 200 | Set-Content (Join-Path $outDir 'pnp-i2c.txt')

# --- MMIO ranges allocated to those devices ---------------------------------
# Maps each PnP device to its memory resources; for an I2C controller in ACPI
# mode this is the DesignWare register block the UEFI driver would use.
Get-CimInstance Win32_PNPAllocatedResource | ForEach-Object {
  $dep = $_.Dependent.DeviceID
  $ant = $_.Antecedent
  if ($ant.PSObject.TypeNames -match 'DeviceMemoryAddress' -or $ant.CimClass.CimClassName -eq 'Win32_DeviceMemoryAddress') {
    "{0}  MEM {1}" -f $dep, $ant.StartingAddress
  }
} | Where-Object { $_ } | Sort-Object |
  Set-Content (Join-Path $outDir 'pnp-resources.txt')

# --- zip --------------------------------------------------------------------
$zip = Join-Path $PSScriptRoot "touch-hwinfo-$product.zip"
if (Test-Path $zip) { Remove-Item $zip -Force }
Compress-Archive -Path "$outDir\*" -DestinationPath $zip
Write-Host "Wrote $zip"
Write-Host 'Attach this zip (or its contents) to the TouchI2cDxe issue/session.'
