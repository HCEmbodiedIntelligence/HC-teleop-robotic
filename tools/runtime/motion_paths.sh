# Use the existing overlay when available, otherwise build inside this checkout.
if [[ -d "${PROJECT_ROOT}/install_motion" && ! -d "${PROJECT_ROOT}/runtime/motion/install" ]]; then
  motion_install_default="${PROJECT_ROOT}/install_motion"
  motion_build_default="${PROJECT_ROOT}/build_motion"
else
  motion_install_default="${PROJECT_ROOT}/runtime/motion/install"
  motion_build_default="${PROJECT_ROOT}/runtime/motion/build"
fi
export HC_MOTION_INSTALL="${HC_MOTION_INSTALL:-${motion_install_default}}"
export HC_MOTION_BUILD="${HC_MOTION_BUILD:-${motion_build_default}}"
