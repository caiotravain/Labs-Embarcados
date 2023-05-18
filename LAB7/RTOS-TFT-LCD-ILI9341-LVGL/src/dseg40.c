/*******************************************************************************
 * Size: 40 px
 * Bpp: 1
 * Opts: 
 ******************************************************************************/
#define LV_LVGL_H_INCLUDE_SIMPLE
#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif

#ifndef DSEG40
#define DSEG40 1
#endif

#if DSEG40

/*-----------------
 *    BITMAPS
 *----------------*/

/*Store the image of the glyphs*/
static LV_ATTRIBUTE_LARGE_CONST const uint8_t glyph_bitmap[] = {
    /* U+002D "-" */
    0x7f, 0xff, 0xef, 0xff, 0xfb, 0xff, 0xfe, 0x0,

    /* U+002E "." */
    0x77, 0xff, 0xf7, 0x0,

    /* U+0030 "0" */
    0xff, 0xff, 0xfe, 0xdf, 0xff, 0xfe, 0xfb, 0xff,
    0xfe, 0xfe, 0x0, 0x0, 0x7f, 0x0, 0x0, 0x7f,
    0x80, 0x0, 0x3f, 0xc0, 0x0, 0x1f, 0xe0, 0x0,
    0xf, 0xf0, 0x0, 0x7, 0xf8, 0x0, 0x3, 0xfc,
    0x0, 0x1, 0xfe, 0x0, 0x0, 0xff, 0x0, 0x0,
    0x7f, 0x80, 0x0, 0x3f, 0xc0, 0x0, 0x1f, 0xe0,
    0x0, 0xf, 0xf0, 0x0, 0x7, 0xf8, 0x0, 0x1,
    0xfc, 0x0, 0x0, 0x3c, 0x0, 0x0, 0x66, 0x0,
    0x0, 0x3c, 0x0, 0x0, 0x3f, 0x80, 0x0, 0x1f,
    0xe0, 0x0, 0xf, 0xf0, 0x0, 0x7, 0xf8, 0x0,
    0x3, 0xfc, 0x0, 0x1, 0xfe, 0x0, 0x0, 0xff,
    0x0, 0x0, 0x7f, 0x80, 0x0, 0x3f, 0xc0, 0x0,
    0x1f, 0xe0, 0x0, 0xf, 0xf0, 0x0, 0x7, 0xf8,
    0x0, 0x3, 0xfc, 0x0, 0x1, 0xfe, 0x0, 0x0,
    0xff, 0x0, 0x0, 0x7f, 0xbf, 0xff, 0xdf, 0xbf,
    0xff, 0xfb, 0xbf, 0xff, 0xff,

    /* U+0031 "1" */
    0x13, 0x37, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xf7, 0x16, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xf3,

    /* U+0032 "2" */
    0xff, 0xff, 0xfe, 0x9f, 0xff, 0xfe, 0xc3, 0xff,
    0xfe, 0xe0, 0x0, 0x0, 0x70, 0x0, 0x0, 0x78,
    0x0, 0x0, 0x3c, 0x0, 0x0, 0x1e, 0x0, 0x0,
    0xf, 0x0, 0x0, 0x7, 0x80, 0x0, 0x3, 0xc0,
    0x0, 0x1, 0xe0, 0x0, 0x0, 0xf0, 0x0, 0x0,
    0x78, 0x0, 0x0, 0x3c, 0x0, 0x0, 0x1e, 0x0,
    0x0, 0xf, 0x0, 0x0, 0x7, 0x80, 0x0, 0x1,
    0xc1, 0xff, 0xff, 0x21, 0xff, 0xff, 0x9, 0xff,
    0xff, 0x7, 0x0, 0x0, 0x3, 0xc0, 0x0, 0x1,
    0xe0, 0x0, 0x0, 0xf0, 0x0, 0x0, 0x78, 0x0,
    0x0, 0x3c, 0x0, 0x0, 0x1e, 0x0, 0x0, 0xf,
    0x0, 0x0, 0x7, 0x80, 0x0, 0x3, 0xc0, 0x0,
    0x1, 0xe0, 0x0, 0x0, 0xf0, 0x0, 0x0, 0x78,
    0x0, 0x0, 0x3c, 0x0, 0x0, 0x1e, 0x0, 0x0,
    0xf, 0x0, 0x0, 0x7, 0xbf, 0xff, 0xc3, 0xbf,
    0xff, 0xf9, 0xbf, 0xff, 0xff,

    /* U+0033 "3" */
    0xff, 0xff, 0xfe, 0x9f, 0xff, 0xfe, 0xc3, 0xff,
    0xfe, 0xe0, 0x0, 0x0, 0x70, 0x0, 0x0, 0x78,
    0x0, 0x0, 0x3c, 0x0, 0x0, 0x1e, 0x0, 0x0,
    0xf, 0x0, 0x0, 0x7, 0x80, 0x0, 0x3, 0xc0,
    0x0, 0x1, 0xe0, 0x0, 0x0, 0xf0, 0x0, 0x0,
    0x78, 0x0, 0x0, 0x3c, 0x0, 0x0, 0x1e, 0x0,
    0x0, 0xf, 0x0, 0x0, 0x7, 0x80, 0x0, 0x1,
    0xc3, 0xff, 0xff, 0x21, 0xff, 0xff, 0x71, 0xff,
    0xff, 0x78, 0x0, 0x0, 0x3c, 0x0, 0x0, 0x1e,
    0x0, 0x0, 0xf, 0x0, 0x0, 0x7, 0x80, 0x0,
    0x3, 0xc0, 0x0, 0x1, 0xe0, 0x0, 0x0, 0xf0,
    0x0, 0x0, 0x78, 0x0, 0x0, 0x3c, 0x0, 0x0,
    0x1e, 0x0, 0x0, 0xf, 0x0, 0x0, 0x7, 0x80,
    0x0, 0x3, 0xc0, 0x0, 0x1, 0xe0, 0x0, 0x0,
    0xf0, 0x0, 0x0, 0x78, 0x7f, 0xff, 0xdc, 0x3f,
    0xff, 0xfa, 0x3f, 0xff, 0xff,

    /* U+0034 "4" */
    0x0, 0x0, 0x0, 0x80, 0x0, 0x0, 0xf0, 0x0,
    0x0, 0x7e, 0x0, 0x0, 0x7f, 0x0, 0x0, 0x7f,
    0x80, 0x0, 0x3f, 0xc0, 0x0, 0x1f, 0xe0, 0x0,
    0xf, 0xf0, 0x0, 0x7, 0xf8, 0x0, 0x3, 0xfc,
    0x0, 0x1, 0xfe, 0x0, 0x0, 0xff, 0x0, 0x0,
    0x7f, 0x80, 0x0, 0x3f, 0xc0, 0x0, 0x1f, 0xe0,
    0x0, 0xf, 0xf0, 0x0, 0x7, 0xf8, 0x0, 0x1,
    0xfd, 0xff, 0xff, 0x3d, 0xff, 0xff, 0x71, 0xff,
    0xff, 0x78, 0x0, 0x0, 0x3c, 0x0, 0x0, 0x1e,
    0x0, 0x0, 0xf, 0x0, 0x0, 0x7, 0x80, 0x0,
    0x3, 0xc0, 0x0, 0x1, 0xe0, 0x0, 0x0, 0xf0,
    0x0, 0x0, 0x78, 0x0, 0x0, 0x3c, 0x0, 0x0,
    0x1e, 0x0, 0x0, 0xf, 0x0, 0x0, 0x7, 0x80,
    0x0, 0x3, 0xc0, 0x0, 0x1, 0xe0, 0x0, 0x0,
    0xf0, 0x0, 0x0, 0x18, 0x0, 0x0, 0x0,

    /* U+0035 "5" */
    0xff, 0xff, 0xfe, 0x5f, 0xff, 0xfe, 0x3b, 0xff,
    0xfe, 0x1e, 0x0, 0x0, 0xf, 0x0, 0x0, 0x7,
    0x80, 0x0, 0x3, 0xc0, 0x0, 0x1, 0xe0, 0x0,
    0x0, 0xf0, 0x0, 0x0, 0x78, 0x0, 0x0, 0x3c,
    0x0, 0x0, 0x1e, 0x0, 0x0, 0xf, 0x0, 0x0,
    0x7, 0x80, 0x0, 0x3, 0xc0, 0x0, 0x1, 0xe0,
    0x0, 0x0, 0xf0, 0x0, 0x0, 0x78, 0x0, 0x0,
    0x3d, 0xff, 0xff, 0x1d, 0xff, 0xff, 0x71, 0xff,
    0xff, 0x78, 0x0, 0x0, 0x3c, 0x0, 0x0, 0x1e,
    0x0, 0x0, 0xf, 0x0, 0x0, 0x7, 0x80, 0x0,
    0x3, 0xc0, 0x0, 0x1, 0xe0, 0x0, 0x0, 0xf0,
    0x0, 0x0, 0x78, 0x0, 0x0, 0x3c, 0x0, 0x0,
    0x1e, 0x0, 0x0, 0xf, 0x0, 0x0, 0x7, 0x80,
    0x0, 0x3, 0xc0, 0x0, 0x1, 0xe0, 0x0, 0x0,
    0xf0, 0x0, 0x0, 0x78, 0x3f, 0xff, 0xdc, 0x3f,
    0xff, 0xfa, 0x3f, 0xff, 0xff,

    /* U+0036 "6" */
    0xff, 0xff, 0xfe, 0x5f, 0xff, 0xfe, 0x3b, 0xff,
    0xfe, 0x1e, 0x0, 0x0, 0xf, 0x0, 0x0, 0x7,
    0x80, 0x0, 0x3, 0xc0, 0x0, 0x1, 0xe0, 0x0,
    0x0, 0xf0, 0x0, 0x0, 0x78, 0x0, 0x0, 0x3c,
    0x0, 0x0, 0x1e, 0x0, 0x0, 0xf, 0x0, 0x0,
    0x7, 0x80, 0x0, 0x3, 0xc0, 0x0, 0x1, 0xe0,
    0x0, 0x0, 0xf0, 0x0, 0x0, 0x78, 0x0, 0x0,
    0x3d, 0xff, 0xff, 0x1d, 0xff, 0xff, 0x79, 0xff,
    0xff, 0x7f, 0x0, 0x0, 0x3f, 0xc0, 0x0, 0x1f,
    0xe0, 0x0, 0xf, 0xf0, 0x0, 0x7, 0xf8, 0x0,
    0x3, 0xfc, 0x0, 0x1, 0xfe, 0x0, 0x0, 0xff,
    0x0, 0x0, 0x7f, 0x80, 0x0, 0x3f, 0xc0, 0x0,
    0x1f, 0xe0, 0x0, 0xf, 0xf0, 0x0, 0x7, 0xf8,
    0x0, 0x3, 0xfc, 0x0, 0x1, 0xfe, 0x0, 0x0,
    0xff, 0x0, 0x0, 0x7f, 0xbf, 0xff, 0xdf, 0xbf,
    0xff, 0xfb, 0xbf, 0xff, 0xff,

    /* U+0037 "7" */
    0x0, 0x0, 0x0, 0x7f, 0xff, 0xff, 0x6f, 0xff,
    0xff, 0x7d, 0xff, 0xff, 0x7f, 0x0, 0x0, 0x3f,
    0x80, 0x0, 0x3f, 0xc0, 0x0, 0x1f, 0xe0, 0x0,
    0xf, 0xf0, 0x0, 0x7, 0xf8, 0x0, 0x3, 0xfc,
    0x0, 0x1, 0xfe, 0x0, 0x0, 0xff, 0x0, 0x0,
    0x7f, 0x80, 0x0, 0x3f, 0xc0, 0x0, 0x1f, 0xe0,
    0x0, 0xf, 0xf0, 0x0, 0x7, 0xf8, 0x0, 0x3,
    0xfc, 0x0, 0x0, 0x7c, 0x0, 0x0, 0x6, 0x0,
    0x0, 0x38, 0x0, 0x0, 0x3c, 0x0, 0x0, 0x1e,
    0x0, 0x0, 0xf, 0x0, 0x0, 0x7, 0x80, 0x0,
    0x3, 0xc0, 0x0, 0x1, 0xe0, 0x0, 0x0, 0xf0,
    0x0, 0x0, 0x78, 0x0, 0x0, 0x3c, 0x0, 0x0,
    0x1e, 0x0, 0x0, 0xf, 0x0, 0x0, 0x7, 0x80,
    0x0, 0x3, 0xc0, 0x0, 0x1, 0xe0, 0x0, 0x0,
    0xf0, 0x0, 0x0, 0x78, 0x0, 0x0, 0x1c, 0x0,
    0x0, 0x2,

    /* U+0038 "8" */
    0xff, 0xff, 0xfe, 0xdf, 0xff, 0xfe, 0xfb, 0xff,
    0xfe, 0xfe, 0x0, 0x0, 0x7f, 0x0, 0x0, 0x7f,
    0x80, 0x0, 0x3f, 0xc0, 0x0, 0x1f, 0xe0, 0x0,
    0xf, 0xf0, 0x0, 0x7, 0xf8, 0x0, 0x3, 0xfc,
    0x0, 0x1, 0xfe, 0x0, 0x0, 0xff, 0x0, 0x0,
    0x7f, 0x80, 0x0, 0x3f, 0xc0, 0x0, 0x1f, 0xe0,
    0x0, 0xf, 0xf0, 0x0, 0x7, 0xf8, 0x0, 0x1,
    0xfd, 0xff, 0xff, 0x3d, 0xff, 0xff, 0x79, 0xff,
    0xff, 0x7f, 0x0, 0x0, 0x3f, 0xc0, 0x0, 0x1f,
    0xe0, 0x0, 0xf, 0xf0, 0x0, 0x7, 0xf8, 0x0,
    0x3, 0xfc, 0x0, 0x1, 0xfe, 0x0, 0x0, 0xff,
    0x0, 0x0, 0x7f, 0x80, 0x0, 0x3f, 0xc0, 0x0,
    0x1f, 0xe0, 0x0, 0xf, 0xf0, 0x0, 0x7, 0xf8,
    0x0, 0x3, 0xfc, 0x0, 0x1, 0xfe, 0x0, 0x0,
    0xff, 0x0, 0x0, 0x7f, 0xbf, 0xff, 0xdf, 0xbf,
    0xff, 0xfb, 0xbf, 0xff, 0xff,

    /* U+0039 "9" */
    0xff, 0xff, 0xfe, 0xdf, 0xff, 0xfe, 0xfb, 0xff,
    0xfe, 0xfe, 0x0, 0x0, 0x7f, 0x0, 0x0, 0x7f,
    0x80, 0x0, 0x3f, 0xc0, 0x0, 0x1f, 0xe0, 0x0,
    0xf, 0xf0, 0x0, 0x7, 0xf8, 0x0, 0x3, 0xfc,
    0x0, 0x1, 0xfe, 0x0, 0x0, 0xff, 0x0, 0x0,
    0x7f, 0x80, 0x0, 0x3f, 0xc0, 0x0, 0x1f, 0xe0,
    0x0, 0xf, 0xf0, 0x0, 0x7, 0xf8, 0x0, 0x1,
    0xfd, 0xff, 0xff, 0x3d, 0xff, 0xff, 0x71, 0xff,
    0xff, 0x78, 0x0, 0x0, 0x3c, 0x0, 0x0, 0x1e,
    0x0, 0x0, 0xf, 0x0, 0x0, 0x7, 0x80, 0x0,
    0x3, 0xc0, 0x0, 0x1, 0xe0, 0x0, 0x0, 0xf0,
    0x0, 0x0, 0x78, 0x0, 0x0, 0x3c, 0x0, 0x0,
    0x1e, 0x0, 0x0, 0xf, 0x0, 0x0, 0x7, 0x80,
    0x0, 0x3, 0xc0, 0x0, 0x1, 0xe0, 0x0, 0x0,
    0xf0, 0x0, 0x0, 0x78, 0x3f, 0xff, 0xdc, 0x3f,
    0xff, 0xfa, 0x3f, 0xff, 0xff
};


/*---------------------
 *  GLYPH DESCRIPTION
 *--------------------*/

static const lv_font_fmt_txt_glyph_dsc_t glyph_dsc[] = {
    {.bitmap_index = 0, .adv_w = 0, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0} /* id = 0 reserved */,
    {.bitmap_index = 0, .adv_w = 522, .box_w = 19, .box_h = 3, .ofs_x = 7, .ofs_y = 18},
    {.bitmap_index = 8, .adv_w = 0, .box_w = 5, .box_h = 5, .ofs_x = -2, .ofs_y = 0},
    {.bitmap_index = 12, .adv_w = 522, .box_w = 25, .box_h = 40, .ofs_x = 4, .ofs_y = 0},
    {.bitmap_index = 137, .adv_w = 522, .box_w = 4, .box_h = 38, .ofs_x = 25, .ofs_y = 2},
    {.bitmap_index = 156, .adv_w = 522, .box_w = 25, .box_h = 40, .ofs_x = 4, .ofs_y = 0},
    {.bitmap_index = 281, .adv_w = 522, .box_w = 25, .box_h = 40, .ofs_x = 4, .ofs_y = 0},
    {.bitmap_index = 406, .adv_w = 522, .box_w = 25, .box_h = 38, .ofs_x = 4, .ofs_y = 1},
    {.bitmap_index = 525, .adv_w = 522, .box_w = 25, .box_h = 40, .ofs_x = 4, .ofs_y = 0},
    {.bitmap_index = 650, .adv_w = 522, .box_w = 25, .box_h = 40, .ofs_x = 4, .ofs_y = 0},
    {.bitmap_index = 775, .adv_w = 522, .box_w = 25, .box_h = 39, .ofs_x = 4, .ofs_y = 2},
    {.bitmap_index = 897, .adv_w = 522, .box_w = 25, .box_h = 40, .ofs_x = 4, .ofs_y = 0},
    {.bitmap_index = 1022, .adv_w = 522, .box_w = 25, .box_h = 40, .ofs_x = 4, .ofs_y = 0}
};

/*---------------------
 *  CHARACTER MAPPING
 *--------------------*/

static const uint8_t glyph_id_ofs_list_0[] = {
    0, 1, 0, 2, 3, 4, 5, 6,
    7, 8, 9, 10, 11
};

/*Collect the unicode lists and glyph_id offsets*/
static const lv_font_fmt_txt_cmap_t cmaps[] =
{
    {
        .range_start = 45, .range_length = 13, .glyph_id_start = 1,
        .unicode_list = NULL, .glyph_id_ofs_list = glyph_id_ofs_list_0, .list_length = 13, .type = LV_FONT_FMT_TXT_CMAP_FORMAT0_FULL
    }
};



/*--------------------
 *  ALL CUSTOM DATA
 *--------------------*/

#if LV_VERSION_CHECK(8, 0, 0)
/*Store all the custom data of the font*/
static  lv_font_fmt_txt_glyph_cache_t cache;
static const lv_font_fmt_txt_dsc_t font_dsc = {
#else
static lv_font_fmt_txt_dsc_t font_dsc = {
#endif
    .glyph_bitmap = glyph_bitmap,
    .glyph_dsc = glyph_dsc,
    .cmaps = cmaps,
    .kern_dsc = NULL,
    .kern_scale = 0,
    .cmap_num = 1,
    .bpp = 1,
    .kern_classes = 0,
    .bitmap_format = 0,
#if LV_VERSION_CHECK(8, 0, 0)
    .cache = &cache
#endif
};


/*-----------------
 *  PUBLIC FONT
 *----------------*/

/*Initialize a public general font descriptor*/
#if LV_VERSION_CHECK(8, 0, 0)
const lv_font_t dseg40 = {
#else
lv_font_t dseg40 = {
#endif
    .get_glyph_dsc = lv_font_get_glyph_dsc_fmt_txt,    /*Function pointer to get glyph's data*/
    .get_glyph_bitmap = lv_font_get_bitmap_fmt_txt,    /*Function pointer to get glyph's bitmap*/
    .line_height = 41,          /*The maximum line height required by the font*/
    .base_line = 0,             /*Baseline measured from the bottom of the line*/
#if !(LVGL_VERSION_MAJOR == 6 && LVGL_VERSION_MINOR == 0)
    .subpx = LV_FONT_SUBPX_NONE,
#endif
#if LV_VERSION_CHECK(7, 4, 0) || LVGL_VERSION_MAJOR >= 8
    .underline_position = -5,
    .underline_thickness = 2,
#endif
    .dsc = &font_dsc           /*The custom font data. Will be accessed by `get_glyph_bitmap/dsc` */
};



#endif /*#if DSEG40*/

