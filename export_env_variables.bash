# set dependencies dir, all 3rdparty dependencies will be downloaded and compiled here
[[ -z ${IVALAB} ]] && GF_SRC_PREFIX="/tmp" || GF_SRC_PREFIX=${IVALAB}
echo ${GF_SRC_PREFIX}
export GF_ORB_SLAM2_SDK=${GF_SRC_PREFIX}/good_feature/SDK/

# set dependencies install dir, all 3rdparty libs will be installed here
[[ -z ${GF_SLAM_OPENSOURCE_ROOT} ]] && GF_LIB_PREFIX="/opt" || GF_LIB_PREFIX=${GF_SLAM_OPENSOURCE_ROOT}
echo ${GF_LIB_PREFIX}
export GF_ORB_SLAM2_ROOT=${GF_LIB_PREFIX}/gf_orb_slam2
# export GF_ORB_SLAM2_ROOT=/opt/gf_orb_slam2