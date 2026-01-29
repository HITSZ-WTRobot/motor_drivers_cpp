/**
 * @file    init-macros.hpp
 * @author  syhanjin
 * @date    2026-01-28
 */
#pragma once

#define static_new(expr)                                                                           \
    (                                                                                              \
            []()                                                                                   \
            {                                                                                      \
                static auto inst = (expr);                                                         \
                return &inst;                                                                      \
            }())
