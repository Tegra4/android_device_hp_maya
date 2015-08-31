# Inherit device configuration for maya.
$(call inherit-product, device/hp/maya/full_maya.mk)

# Inherit some common CM stuff.
$(call inherit-product, vendor/cm/config/common_full_tablet_wifionly.mk)

PRODUCT_BUILD_PROP_OVERRIDES += \
    PRODUCT_NAME=maya \
    TARGET_DEVICE=maya \
    BUILD_FINGERPRINT="hp/maya/3645:4.3/4.3-17r20-05-24/17r20-05-24:user/release-keys" \
    PRIVATE_BUILD_DESC="maya-user 4.3 4.3-17r20-05-24 17r20-05-24 release-keys"

PRODUCT_NAME := cm_maya
