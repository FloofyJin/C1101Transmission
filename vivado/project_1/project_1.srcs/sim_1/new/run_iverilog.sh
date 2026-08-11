#!/usr/bin/env bash
# Run the RTL testbenches with Icarus Verilog (no Vivado needed).
#
#   ./run_iverilog.sh
#
# These sims prove the FPGA emits the right bytes in the right order with the
# right CSn framing. They CANNOT tell you whether the register values are
# correct for the air interface -- only hardware can.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
src="$here/../../sources_1/new"
out="$(mktemp -d)"
trap 'rm -rf "$out"' EXIT

rtl=(
    "$src/cc1101_pkg.sv"
    "$src/SPIMaster.sv"
    "$src/CC1101Driver.sv"
    "$src/ConfigSeq.sv"
    "$src/TxSeq.sv"
    "$src/RxSeq.sv"
    "$src/PointRam.sv"
    "$src/Mcp4922Driver.sv"
    "$src/ScanoutEngine.sv"
    "$src/topRF.sv"
    "$here/cc1101_model.sv"
)

fail=0
for tb in tb_topRF tb_topRF_timeout tb_rx_points tb_mcp4922 tb_scanout; do
    echo "=============== $tb ==============="
    iverilog -g2012 -o "$out/$tb.vvp" -s "$tb" "${rtl[@]}" "$here/$tb.sv"
    vvp "$out/$tb.vvp" || fail=1
done

exit $fail
