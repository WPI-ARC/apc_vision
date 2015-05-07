#include "ObjectIDs.h"

uint64_t string_to_id(std::string str) {
    if(str == "oreo_mega_stuf") {
        return OREO_MEGA_STUF;
    } else if(str == "champion_copper_plus_spark_plug") {
        return CHAMPION_COPPER_PLUS_SPARK_PLUG;
    } else if(str == "expo_dry_erase_board_eraser") {
        return EXPO_DRY_ERASE_BOARD_ERASER;
    } else if(str == "genuine_joe_plastic_stir_sticks") {
        return GENUINE_JOE_PLASTIC_STIR_STICKS;
    } else if(str == "munchkin_white_hot_duck_bath_toy") {
        return MUNCHKIN_WHITE_HOT_DUCK_BATH_TOY;
    } else if(str == "crayola_64_ct") {
        return CRAYOLA_64_CT;
    } else if(str == "mommys_helper_outlet_plugs") {
        return MOMMYS_HELPER_OUTLET_PLUGS;
    } else if(str == "sharpie_accent_tank_style_highlighters") {
        return SHARPIE_ACCENT_TANK_STYLE_HIGHLIGHTERS;
    } else if(str == "stanley_66_052") {
        return STANLEY_66_052;
    } else if(str == "safety_works_safety_glasses") {
        return SAFETY_WORKS_SAFETY_GLASSES;
    } else if(str == "cheezit_big_original") {
        return CHEEZIT_BIG_ORIGINAL;
    } else if(str == "paper_mate_12_count_mirado_black_warrior") {
        return PAPER_MATE_12_COUNT_MIRADO_BLACK_WARRIOR;
    } else if(str == "feline_greenies_dental_treats") {
        return FELINE_GREENIES_DENTAL_TREATS;
    } else if(str == "elmers_washable_no_run_school_glue") {
        return ELMERS_WASHABLE_NO_RUN_SCHOOL_GLUE;
    } else if(str == "mead_index_cards") {
        return MEAD_INDEX_CARDS;
    } else if(str == "rolodex_jumbo_pencil_cup") {
        return ROLODEX_JUMBO_PENCIL_CUP;
    } else if(str == "first_years_take_and_toss_straw_cup") {
        return FIRST_YEARS_TAKE_AND_TOSS_STRAW_CUP;
    } else if(str == "highland_6539_self_stick_notes") {
        return HIGHLAND_6539_SELF_STICK_NOTES;
    } else if(str == "mark_twain_huckleberry_finn") {
        return MARK_TWAIN_HUCKLEBERRY_FINN;
    } else if(str == "kyjen_squeakin_eggs_plush_puppies") {
        return KYJEN_SQUEAKIN_EGGS_PLUSH_PUPPIES;
    } else if(str == "kong_sitting_frog_dog_toy") {
        return KONG_SITTING_FROG_DOG_TOY;
    } else if(str == "kon_air_dog_squeakair_tennis_ball") {
        return KON_AIR_DOG_SQUEAKAIR_TENNIS_BALL;
    } else if(str == "dr_browns_bottle_brush") {
        return DR_BROWNS_BOTTLE_BRUSH;
    } else if(str == "kong_duck_dog_toy") {
        return KONG_DUCK_DOG_TOY;
    } else if(str == "laugh_out_loud_joke_book") {
        return LAUGH_OUT_LOUD_JOKE_BOOK;
    } else {
        return INVALID_ID;
    }
}

std::string id_to_string(uint64_t id) {
    switch(id) {
    case OREO_MEGA_STUF:
        return std::string("oreo_mega_stuf");
    case CHAMPION_COPPER_PLUS_SPARK_PLUG:
        return std::string("champion_copper_plus_spark_plug");
    case EXPO_DRY_ERASE_BOARD_ERASER:
        return std::string("expo_dry_erase_board_eraser");
    case GENUINE_JOE_PLASTIC_STIR_STICKS:
        return std::string("genuine_joe_plastic_stir_sticks");
    case MUNCHKIN_WHITE_HOT_DUCK_BATH_TOY:
        return std::string("munchkin_white_hot_duck_bath_toy");
    case CRAYOLA_64_CT:
        return std::string("crayola_64_ct");
    case MOMMYS_HELPER_OUTLET_PLUGS:
        return std::string("mommys_helper_outlet_plugs");
    case SHARPIE_ACCENT_TANK_STYLE_HIGHLIGHTERS:
        return std::string("sharpie_accent_tank_style_highlighters");
    case STANLEY_66_052:
        return std::string("stanley_66_052");
    case SAFETY_WORKS_SAFETY_GLASSES:
        return std::string("safety_works_safety_glasses");
    case CHEEZIT_BIG_ORIGINAL:
        return std::string("cheezit_big_original");
    case PAPER_MATE_12_COUNT_MIRADO_BLACK_WARRIOR:
        return std::string("paper_mate_12_count_mirado_black_warrior");
    case FELINE_GREENIES_DENTAL_TREATS:
        return std::string("feline_greenies_dental_treats");
    case ELMERS_WASHABLE_NO_RUN_SCHOOL_GLUE:
        return std::string("elmers_washable_no_run_school_glue");
    case MEAD_INDEX_CARDS:
        return std::string("mead_index_cards");
    case ROLODEX_JUMBO_PENCIL_CUP:
        return std::string("rolodex_jumbo_pencil_cup");
    case FIRST_YEARS_TAKE_AND_TOSS_STRAW_CUP:
        return std::string("first_years_take_and_toss_straw_cup");
    case HIGHLAND_6539_SELF_STICK_NOTES:
        return std::string("highland_6539_self_stick_notes");
    case MARK_TWAIN_HUCKLEBERRY_FINN:
        return std::string("mark_twain_huckleberry_finn");
    case KYJEN_SQUEAKIN_EGGS_PLUSH_PUPPIES:
        return std::string("kyjen_squeakin_eggs_plush_puppies");
    case KONG_SITTING_FROG_DOG_TOY:
        return std::string("kong_sitting_frog_dog_toy");
    case KON_AIR_DOG_SQUEAKAIR_TENNIS_BALL:
        return std::string("kon_air_dog_squeakair_tennis_ball");
    case DR_BROWNS_BOTTLE_BRUSH:
        return std::string("dr_browns_bottle_brush");
    case KONG_DUCK_DOG_TOY:
        return std::string("kong_duck_dog_toy");
    case LAUGH_OUT_LOUD_JOKE_BOOK:
        return std::string("laugh_out_loud_joke_book");
    default:
        return std::string("invalid");
    }
}
