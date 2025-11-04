import ctypes
import os
import sys
from ament_index_python.packages import get_package_share_directory
import logging
logging.basicConfig(level=logging.INFO)

# 取得安裝後的 share 目錄
try:
    pkg_share_dir = get_package_share_directory('player_bridge')
    lib_path = os.path.join(pkg_share_dir, 'client_lib')
except Exception:
    # 若直接執行（非安裝狀態）
    pkg_dir = os.path.dirname(os.path.realpath(__file__))
    lib_path = os.path.join(os.path.dirname(pkg_dir), 'client_lib')

# 加入到 sys.path 讓 python 能 import
if os.path.isdir(lib_path) and lib_path not in sys.path:
    sys.path.insert(0, lib_path)

# 加入動態連結庫搜尋路徑
os.environ['LD_LIBRARY_PATH'] = os.environ.get('LD_LIBRARY_PATH', '') + ':' + lib_path

# 確認目前路徑
logging.debug("✅ client_lib path: %s", lib_path)

# Preload bundled native libraries that the SWIG extension depends on.
# Some of the binary .so files (built against an older Python ABI) may
# reference libpython3.6m.so.1.0. The system has Python 3.10, so the
# dynamic loader can't find that library. If a copy of libpython3.6m
# is shipped in `client_lib/`, preload it with RTLD_GLOBAL so the
# subsequent import of the SWIG _playercpp extension can resolve symbols.
# try:
#     # Look for the old libpython in the client_lib folder and preload it.
#     possible = [
#         os.path.join(lib_path, 'libpython3.6m.so.1.0'),
#         os.path.join(lib_path, 'libpython3.6m.so'),
#     ]
#     # Also attempt explicit preload of libplayerc++ which _playercpp depends on
#     possible.extend([
#         os.path.join(lib_path, 'libplayerc++.so.3.1'),
#         os.path.join(lib_path, 'libplayerc++.so'),
#     ])
#     loaded = []
#     for p in possible:
#         if os.path.exists(p):
#             try:
#                 # Use RTLD_GLOBAL to make symbols available to subsequently
#                 # loaded extensions. Use mode value from ctypes if available.
#                 mode = getattr(ctypes, 'RTLD_GLOBAL', None)
#                 if mode is not None:
#                     ctypes.CDLL(p, mode)
#                 else:
#                     ctypes.CDLL(p)
#                 loaded.append(p)
#             except OSError:
#                 # ignore load errors and continue
#                 pass
#     if loaded:
#         print('✅ preloaded native libs:', ','.join(loaded))
# except Exception as _e:
#     # Don't fail here; fall back to normal import and let import raise a helpful error.
#     print('⚠️ preloading native libs failed:', str(_e))

# Verbose explicit attempt to preload libplayerc++ variants so we can
# see whether they are actually present and loadable before importing
# the SWIG extension.
# Try to preload player libraries in a dependency-aware order so that
# lower-level libs (libplayerc) are loaded before libplayerc++ which
# depends on them. If a load fails, print ldd output to help diagnose
# missing transitive dependencies.
# dep_order = [
#     'libplayerc.so.3.1', 'libplayerc.so',
#     'libplayercommon.so.3.1', 'libplayercommon.so',
#     'libplayercore.so.3.1', 'libplayercore.so',
#     'libplayerinterface.so.3.1', 'libplayerinterface.so',
#     'libplayerjpeg.so.3.1', 'libplayerjpeg.so',
#     'libplayerreplace.so.3.1', 'libplayerreplace.so',
#     'libplayerwkb.so.3.1', 'libplayerwkb.so',
#     'libplayerc++.so.3.1', 'libplayerc++.so',
#     'libpython3.6m.so.1.0', 'libpython3.6m.so'
# ]
dep_order = [
    'libplayercommon.so.3.1', 'libplayercommon.so',
    'libjemalloc.so.2',
    'libplayerinterface.so.3.1', 'libplayerinterface.so',
    'libplayerwkb.so.3.1', 'libplayerwkb.so',
    'liblzma.so.5',
    'libz.so.1',
    'libgcc_s.so.1',
    'libgeos-3.6.2.so', 'libgeos_c.so.1',
    'libjpeg.so.8',
    'libplayerjpeg.so.3.1', 'libplayerjpeg.so',
    'libplayerc.so.3.1', 'libplayerc.so',
    'libplayercore.so.3.1', 'libplayercore.so',
    'libplayerreplace.so.3.1', 'libplayerreplace.so',
    
    'libboost_system.so.1.65.1', 'libboost_thread.so.1.65.1',
    'libplayerc++.so.3.1', 'libplayerc++.so',
    'libpython3.6m.so.1.0', 'libpython3.6m.so'
]
loaded_deps = []
failed_deps = []
import subprocess

for name in dep_order:
    p = os.path.join(lib_path, name)
    if not os.path.exists(p):
        continue
    try:
        mode = getattr(ctypes, 'RTLD_GLOBAL', None)
        if mode is not None:
            ctypes.CDLL(p, mode)
        else:
            ctypes.CDLL(p)
        loaded_deps.append(p)
        logging.debug('✅ preloaded: %s', name)
    except OSError as e:
        failed_deps.append((p, str(e)))
        logging.warning('⚠️ failed to preload %s: %s', p, e)
        # Try to run ldd on the file to show missing transitive dependencies
        try:
            out = subprocess.check_output(['ldd', p], stderr=subprocess.STDOUT, text=True)
            logging.debug('ldd output for %s ->\n%s', name, out)
        except Exception as le:
            logging.warning('Could not run ldd for %s: %s', p, le)

if loaded_deps:
    logging.debug('✅ preloaded native libs (ordered): %s', ','.join(loaded_deps))
if failed_deps:
    logging.warning('⚠️ some native libs failed to preload; see messages above')