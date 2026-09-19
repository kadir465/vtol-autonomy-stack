#!/bin/bash
# ═══════════════════════════════════════════════════════════════
#  VTOL Clean Build Script
#  ─────────────────────────────────────────────────────────────
#  Bu script install/ ve build/ cache'ini temizleyip
#  --symlink-install ile yeniden derler.
#  Symlink sayesinde src/ içindeki değişiklikler anında aktif olur.
#
#  Kullanım:
#    cd ~/vtol_ws
#    bash rebuild_clean.sh
# ═══════════════════════════════════════════════════════════════

set -e

WORKSPACE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$WORKSPACE_DIR"

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  VTOL Clean Build — Cache Temizleme + Symlink Build ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""

# 1. Eski cache'i tamamen sil
echo "[1/3] Eski build ve install klasörleri siliniyor..."
rm -rf build/ install/ log/
echo "      ✓ Temizlendi."

# 2. ROS 2 ortamını kaynak et
echo "[2/3] ROS 2 ortamı yükleniyor..."
source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/foxy/setup.bash 2>/dev/null || {
    echo "HATA: ROS 2 ortamı bulunamadı!"
    exit 1
}
echo "      ✓ ROS 2 ortamı yüklendi."

# 3. Symlink install ile derle
echo "[3/3] colcon build --symlink-install çalıştırılıyor..."
colcon build --symlink-install
echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  ✓ BUILD BAŞARILI!                                  ║"
echo "║                                                     ║"
echo "║  Artık src/ içindeki değişiklikler otomatik aktif.   ║"
echo "║  Çalıştırmadan önce:                                ║"
echo "║    source install/setup.bash                        ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
