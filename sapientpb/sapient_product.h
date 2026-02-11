#ifndef SAPIENT_PRODUCT_H
#define SAPIENT_PRODUCT_H

#include <string>

static inline std::string sapient_product_short_name()
{
    // 对于 acur101_ps_linux 项目，产品型号固定为 SDH100
    // - Registration.short_name 需要为 "SDH100"
    // - Registration.name 需要为 "Skyfend SDH100"（见下方 display_name 实现）
    return "SDH100";
}

/**
 * @brief 获取产品展示名称（用于 Registration.name）
 */
static inline std::string sapient_product_display_name()
{
    return std::string("Skyfend ") + sapient_product_short_name();
}

#endif // SAPIENT_PRODUCT_H


