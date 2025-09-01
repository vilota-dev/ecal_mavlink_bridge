# -------------------------------
# Generic CPack config for .deb
# -------------------------------

# 1) 基础变量：从 project() 继承，必要时可手动 set 覆盖
if(NOT DEFINED PROJECT_NAME)
  set(PROJECT_NAME "MyProject")
endif()
if(NOT DEFINED PROJECT_VERSION)
  # 回退：0.0.0
  set(PROJECT_VERSION "0.0.0")
endif()

# 2) 小写名与 Git 信息
string(TOLOWER "${PROJECT_NAME}" PROJECT_NAME_LOWERCASE)

# git short hash
execute_process(
  COMMAND git log -1 --format=%h
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  OUTPUT_VARIABLE GIT_HASH
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET
)
if(NOT GIT_HASH)
  set(GIT_HASH "nogit")
endif()

# git 是否 dirty
execute_process(
  COMMAND git diff --quiet HEAD
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  RESULT_VARIABLE GIT_DIFF_RESULT
  ERROR_QUIET
)
if(GIT_DIFF_RESULT EQUAL 0)
  set(GIT_HASH_TAG "${GIT_HASH}")
else()
  set(GIT_HASH_TAG "${GIT_HASH}-dirty")
endif()

# 打包日期
string(TIMESTAMP BUILD_DATE "%Y%m%d")

# 3) 发行版 / 体系结构探测（尽量鲁棒）
# 架构：dpkg 优先，回退 uname -m -> dpkg 名称
set(_ARCH "")
find_program(DPKG_PROGRAM dpkg)
if(DPKG_PROGRAM)
  execute_process(
    COMMAND ${DPKG_PROGRAM} --print-architecture
    OUTPUT_VARIABLE _ARCH
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
endif()
if(NOT _ARCH)
  execute_process(COMMAND uname -m OUTPUT_VARIABLE _ARCH OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(_ARCH STREQUAL "x86_64")
    set(_ARCH "amd64")
  elseif(_ARCH MATCHES "aarch64|arm64")
    set(_ARCH "arm64")
  elseif(_ARCH MATCHES "^armv7" OR _ARCH STREQUAL "armv7l")
    set(_ARCH "armhf")
  endif()
endif()

# 发行版 ID/Release（仅用于文件名和可选依赖映射）
set(LSB_RELEASE_ID_SHORT "Unknown")
set(LSB_RELEASE_RELEASE_SHORT "Unknown")
find_program(LSB_RELEASE_PROGRAM lsb_release)
if(LSB_RELEASE_PROGRAM)
  execute_process(COMMAND ${LSB_RELEASE_PROGRAM} -is
                  OUTPUT_VARIABLE LSB_RELEASE_ID_SHORT
                  OUTPUT_STRIP_TRAILING_WHITESPACE)
  execute_process(COMMAND ${LSB_RELEASE_PROGRAM} -rs
                  OUTPUT_VARIABLE LSB_RELEASE_RELEASE_SHORT
                  OUTPUT_STRIP_TRAILING_WHITESPACE)
endif()

# 4) 依赖处理（推荐：自动生成）
# 开启 dpkg-shlibdeps 自动分析共享库依赖；必要时可追加手工依赖
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS YES)   # 自动 so 依赖
# 可覆盖/追加：-DDEBIAN_EXTRA_DEPENDS="libtbb12, libopencv-core4.5d"
set(DEBIAN_EXTRA_DEPENDS "")

# 可选：按发行版映射少量已知依赖（示例，可按需修改/扩展）
set(_DEB_DEPENDS "")
if(LSB_RELEASE_ID_SHORT STREQUAL "Ubuntu" AND LSB_RELEASE_RELEASE_SHORT STREQUAL "20.04")
  set(_DEB_DEPENDS "${_DEB_DEPENDS}")
elseif(LSB_RELEASE_ID_SHORT STREQUAL "Ubuntu" AND LSB_RELEASE_RELEASE_SHORT STREQUAL "22.04")
  set(_DEB_DEPENDS "${_DEB_DEPENDS}")
elseif(LSB_RELEASE_ID_SHORT STREQUAL "Ubuntu" AND LSB_RELEASE_RELEASE_SHORT STREQUAL "24.04")
  set(_DEB_DEPENDS "${_DEB_DEPENDS}")
elseif(LSB_RELEASE_ID_SHORT STREQUAL "Debian" AND LSB_RELEASE_RELEASE_SHORT STREQUAL "12")
  set(_DEB_DEPENDS "${_DEB_DEPENDS}")
endif()

# 合并自动依赖与手工依赖（手工用逗号分隔）
if(DEBIAN_EXTRA_DEPENDS)
  if(_DEB_DEPENDS)
    set(_DEB_DEPENDS "${_DEB_DEPENDS}, ${DEBIAN_EXTRA_DEPENDS}")
  else()
    set(_DEB_DEPENDS "${DEBIAN_EXTRA_DEPENDS}")
  endif()
endif()

# 5) 版本号：MAJOR.MINOR.PATCH-日期-githash-distro
# 你也可以按需裁剪
if(DEFINED PROJECT_VERSION_PATCH)
  set(_VER_PATCH "${PROJECT_VERSION_PATCH}")
else()
  # 若 project() 给的是三段式，CMake 会有 *_PATCH；否则置 0
  set(_VER_PATCH "0")
endif()

set(_VERSION_FULL
    "${PROJECT_VERSION}-${BUILD_DATE}-${GIT_HASH_TAG}-${LSB_RELEASE_ID_SHORT}-${LSB_RELEASE_RELEASE_SHORT}")

# 6) 通用 CPack 变量
set(CPACK_VERBATIM_VARIABLES YES)
set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_VENDOR "Your Company")
set(CPACK_PACKAGE_CONTACT "you@example.com")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "you@example.com")  # 为兼容旧字段
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "${PROJECT_NAME} binaries and resources")
set(CPACK_PACKAGE_DESCRIPTION "${PROJECT_NAME} software package")
set(CPACK_PACKAGE_HOMEPAGE_URL "https://example.com")

# 前缀：/opt/<proj>（可用 -DCPACK_PACKAGING_INSTALL_PREFIX=... 覆盖）
if(NOT DEFINED CPACK_PACKAGING_INSTALL_PREFIX)
  set(CPACK_PACKAGING_INSTALL_PREFIX "/opt/${PROJECT_NAME_LOWERCASE}")
endif()

# 许可文件（不存在时不报错）
if(EXISTS "${CMAKE_SOURCE_DIR}/LICENSE")
  set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_SOURCE_DIR}/LICENSE")
endif()

# 组件：默认把所有组件打进一个包；也支持多组件（见下方示例）
set(CPACK_COMPONENTS_ALL applications libraries runtime)
set(CPACK_COMPONENTS_GROUPING ALL_COMPONENTS_IN_ONE)

# 7) DEB 专有设置
set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "${_ARCH}")
if(_DEB_DEPENDS)
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "${_DEB_DEPENDS}")
endif()
set(CPACK_DEBIAN_COMPRESSION_TYPE "xz")
set(CPACK_DEBIAN_PACKAGE_SECTION "utils")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_DEBIAN_PACKAGE_GENERATE_SHLIBS YES)  # 生成 shlibs（与 SHLIBDEPS 配合良好）

# 可选：preinst/postinst/prerm/postrm 脚本（放到 cmake/debian/ 中）
# set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA
#     "${CMAKE_SOURCE_DIR}/cmake/debian/preinst;${CMAKE_SOURCE_DIR}/cmake/debian/postinst;${CMAKE_SOURCE_DIR}/cmake/debian/prerm;${CMAKE_SOURCE_DIR}/cmake/debian/postrm")

# 8) 文件名：<name>-<semver+meta>-<arch>-<BuildType>
# 例如 myapp-1.2.3-20250829-a1b2c3-Ubuntu-22.04-amd64-Release.deb
# 注意：CPack 会在 include(CPack) 之后最终重写文件名；我们先给一个期望值
set(CPACK_PACKAGE_VERSION "${_VERSION_FULL}")
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()
set(CPACK_PACKAGE_FILE_NAME
  "${PROJECT_NAME_LOWERCASE}-${_VERSION_FULL}-${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}-${CMAKE_BUILD_TYPE}")

# 9) 校验、签名（可选）
set(CPACK_PACKAGE_CHECKSUM "SHA256")
# 若需要 deb 签名，可配：CPACK_DEBIAN_PACKAGE_SIGNING_KEY / DEB_SIGNING_ID（外部脚本更常见）

# 10) include(CPack) 放最后
include(CPack)