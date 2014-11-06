#define NO_SEGMENTATION -1
#define REGION_GROWTH_SEGMENTATION 0
#define EUCLIDEAN_SEGMENTATION 1
#define RGB_BLACK (0x00 | (0x00<<8) | (0x00<<16))

#define MIN_RG_KSEARCH_VALUE 1
#define MAX_RG_KSEARCH_VALUE 100
#define MIN_REMOVESEGMENTS_VALUE 10
#define MAX_REMOVESEGMENTS_VALUE 10000
#define MIN_RG_MINCLUSTER 10
#define MAX_RG_MINCLUSTER 10000
#define MIN_RG_MAXCLUSTER 1 // multiply by 1000
#define MAX_RG_MAXCLUSTER 1000 // multiply by 1000
#define MIN_RG_NEIGHBORS 5
#define MAX_RG_NEIGHBORS 100
#define MIN_RG_SMOOTHNESS 10 // divide by 10
#define MAX_RG_SMOOTHNESS 200 // divide by 10
#define MIN_RG_CURVATURE 1 // divide by 10
#define MAX_RG_CURVATURE 100 // divide by 10
#define MIN_FILLHOLES 1 // multiply by 100
#define MAX_FILLHOLES 1000 // multiply by 100

#define IDC_CHECK_SHOWBB                1093
#define IDC_CHECK_WIREFRAME             1094
#define IDC_CHECK_RG_ESTIMATENORMALS    1107
#define IDC_CHECK_FREECAMERA            1108
#define IDC_CHECK_COLORSELECTION        1109
#define IDC_CHECK_HELPINGVISUALS        1112

#define IDC_BUTTON_RESETCAMERA          1068
#define IDC_BUTTON_EXPORT               1070
#define IDC_BUTTON_SETBACKGROUND        1073
#define IDC_BUTTON_REGION_GROWTH_SEGMENTATION 1074
#define IDC_BUTTON_RG_PREVIEW           1092
#define IDC_BUTTON_RG_EXTERNALPREVIEW   1095
#define IDC_BUTTON_RG_RESETVALUES       1099
#define IDC_BUTTON_FILLHOLES            1105
#define IDC_BUTTON_WALL                 1110
#define IDC_BUTTON_RESETWALL            1111
#define IDC_BUTTON_MLS                  1113
#define IDC_BUTTON_REMOVESEGMENTS       1114
#define IDC_BUTTON_CLEANMESH            1117

#define IDC_EDIT_BACKGROUND_RED         1067
#define IDC_EDIT_BACKGROUND_GREEN       1071
#define IDC_EDIT_BACKGROUND_BLUE        1072
#define IDC_EDIT_RG_KSEARCHVALUE        1096
#define IDC_EDIT_RG_MINCLUSTERSIZE      1097
#define IDC_EDIT_RG_MAXCLUSTERSIZE      1098
#define IDC_EDIT_FILLHOLES              1101
#define IDC_EDIT_RG_NON                 1102
#define IDC_EDIT_RG_SMOOTHNESS          1103
#define IDC_EDIT_RG_CURVATURE           1104
#define IDC_EDIT_REMOVESEGMENTS         1115

#define IDC_SLIDER_RG_KSEARCH_VALUE     1069
#define IDC_SLIDER_RG_MINCLUSTERSIZE    1075
#define IDC_SLIDER_RG_MAXCLUSTERSIZE    1076
#define IDC_SLIDER_RG_NON               1077
#define IDC_SLIDER_RG_SMOOTHNESS        1078
#define IDC_SLIDER_RG_CURVATURE         1079
#define IDC_SLIDER_FILLHOLES            1100
#define IDC_SLIDER_REMOVESEGMENTS       1116

#define IDC_IM_STATUS                   1106

#define IDC_TEXT_RG_KSEARCH_VALUE       1080
#define IDC_TEXT_RG_MINCLUSTERSIZE      1081
#define IDC_TEXT_RG_MAXCLUSTERSIZE      1082
#define IDC_TEXT_RG_NON                 1083
#define IDC_TEXT_RG_SMOOTHNESS          1084
#define IDC_TEXT_RG_CURVATURE           1085

//text descriptions for rg slider values
#define IDC_TEXT                        1086
#define IDC_TEXT2                       1087
#define IDC_TEXT3                       1088
#define IDC_TEXT4                       1089
#define IDC_TEXT5                       1090
#define IDC_TEXT6                       1091