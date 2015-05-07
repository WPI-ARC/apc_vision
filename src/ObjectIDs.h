#pragma once
#include <stdint.h>
#include <string>

const uint64_t INVALID_ID                                 = 0;
const uint64_t OREO_MEGA_STUF                             = 1<<0;
const uint64_t CHAMPION_COPPER_PLUS_SPARK_PLUG            = 1<<1;
const uint64_t EXPO_DRY_ERASE_BOARD_ERASER                = 1<<2;
const uint64_t GENUINE_JOE_PLASTIC_STIR_STICKS            = 1<<3;
const uint64_t MUNCHKIN_WHITE_HOT_DUCK_BATH_TOY           = 1<<4;
const uint64_t CRAYOLA_64_CT                              = 1<<5;
const uint64_t MOMMYS_HELPER_OUTLET_PLUGS                 = 1<<6;
const uint64_t SHARPIE_ACCENT_TANK_STYLE_HIGHLIGHTERS     = 1<<7;
const uint64_t STANLEY_66_052                             = 1<<8;
const uint64_t SAFETY_WORKS_SAFETY_GLASSES                = 1<<9;
const uint64_t CHEEZIT_BIG_ORIGINAL                       = 1<<10;
const uint64_t PAPER_MATE_12_COUNT_MIRADO_BLACK_WARRIOR   = 1<<11;
const uint64_t FELINE_GREENIES_DENTAL_TREATS              = 1<<12;
const uint64_t ELMERS_WASHABLE_NO_RUN_SCHOOL_GLUE         = 1<<13;
const uint64_t MEAD_INDEX_CARDS                           = 1<<14;
const uint64_t ROLODEX_JUMBO_PENCIL_CUP                   = 1<<15;
const uint64_t FIRST_YEARS_TAKE_AND_TOSS_STRAW_CUP        = 1<<16;
const uint64_t HIGHLAND_6539_SELF_STICK_NOTES             = 1<<17;
const uint64_t MARK_TWAIN_HUCKLEBERRY_FINN                = 1<<18;
const uint64_t KYJEN_SQUEAKIN_EGGS_PLUSH_PUPPIES          = 1<<19;
const uint64_t KONG_SITTING_FROG_DOG_TOY                  = 1<<20;
const uint64_t KON_AIR_DOG_SQUEAKAIR_TENNIS_BALL          = 1<<21;
const uint64_t DR_BROWNS_BOTTLE_BRUSH                     = 1<<22;
const uint64_t KONG_DUCK_DOG_TOY                          = 1<<23;
const uint64_t LAUGH_OUT_LOUD_JOKE_BOOK                   = 1<<24;

uint64_t string_to_id(std::string str);
std::string id_to_string(uint64_t id);
