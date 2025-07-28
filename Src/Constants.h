#ifndef GL_CONST_H
#define GL_CONST_H

// ======================
// JSON Tags - Structure
// ======================

#define CNS_TAG_ROOT        "root"

#define CNS_TAG_MAP         "map"
#define CNS_TAG_WIDTH       "width"
#define CNS_TAG_HEIGHT      "height"
#define CNS_TAG_GRID        "grid"
#define CNS_TAG_ROW         "row"

#define CNS_TAG_AGENTS      "agents"
#define CNS_TAG_START       "start"
#define CNS_TAG_FINISH      "finish"
#define CNS_TAG_STX         "startx"
#define CNS_TAG_STY         "starty"
#define CNS_TAG_FINX        "finishx"
#define CNS_TAG_FINY        "finishy"

#define CNS_TAG_ALG         "algorithm"
#define CNS_TAG_MT          "metrictype"
#define CNS_T_MAX           "T_max"

#define CNS_TAG_LOG         "log"
#define CNS_TAG_PATH        "path"
#define CNS_TAG_SUM         "summary"
#define CNS_TAG_AGENTS_PATHS "paths"


// ==========================
// JSON Attributes in Output
// ==========================

#define CNS_TAG_ATTR_NUMOFSTEPS     "numberofsteps"
#define CNS_TAG_ATTR_NODESCREATED   "nodescreated"
#define CNS_TAG_ATTR_LENGTH         "length"
#define CNS_TAG_ATTR_LENGTH_SCALED  "length_scaled"
#define CNS_TAG_ATTR_TIME           "time"

#define CNS_TAG_ATTR_X              "x"
#define CNS_TAG_ATTR_Y              "y"
#define CNS_TAG_ATTR_T_Lower        "start_time"
#define CNS_TAG_ATTR_T_Upper        "end_time"
#define CNS_TAG_VEL                 "velocity"
#define CNS_TAG_ATTR_G              "g"

#define CNS_TAG_ATTR_STX            "start.x"
#define CNS_TAG_ATTR_STY            "start.y"
#define CNS_TAG_ATTR_FINX           "finish.x"
#define CNS_TAG_ATTR_FINY           "finish.y"


// ===================
// Search Parameters
// ===================

#define CNS_SP_ST_PBS      "pbs"

#define CN_MXO                  4
#define CN_MXV                  2

#define CN_SEARCH_PARAMS_COUNT  2
#define CN_SP_MT                0
#define CN_T_MAX                1

#define CNS_SP_MT_DIAG         "diagonal"
#define CNS_SP_MT_MANH         "manhattan"
#define CNS_SP_MT_EUCL         "euclidean"
#define CNS_SP_MT_CHEB         "chebyshev"

#define CN_SP_MT_DIAG          0
#define CN_SP_MT_MANH          1
#define CN_SP_MT_EUCL          2
#define CN_SP_MT_CHEB          3


// ==========
// Grid Cell
// ==========

#define CN_GC_NOOBS  0
#define CN_GC_OBS    1


// =======
// Other
// =======

#define CNS_OTHER_PATHSELECTION     '*'
#define CNS_OTHER_MATRIXSEPARATOR   ','

#endif