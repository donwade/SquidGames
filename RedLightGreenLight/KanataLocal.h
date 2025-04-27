
typedef struct GPS_ENTRY 
{        float lat; 
         float lng; 
         short bearing; 
         const char *street1; 
         const char *street2;
};

GPS_ENTRY gpsLookup[] = 
{
    { +45.2967381, -75.8533438 ,  233    ,"SteepleChase",   "PineHill" },
    { +45.3007901, -75.8543413 ,  155    ,"SteepleChase",   "Mattawa" },
    { +45.2990318, -75.8586619 ,  44     ,"SteepleChase",   "Bonnechere" },
    { +45.2971780, -75.8599713 ,  24     ,"SteepleChase",   "SpringWater" },
    { +45.2917147, -75.8586298 ,  340    ,"SteepleChase",   "StoneHaven" },
    { +45.2902267, -75.8514189 ,  294    ,"StoneHaven",     "Furlong" },
    { +45.2841335, -75.8458727 ,  348    ,"StoneHaven",     "BridlePark" },
    { +45.2833978, -75.8457087 ,  351    ,"StoneHaven",     "Tandalee" },
    { +45.2909958, -75.8575216 ,  89     ,"StoneHaven",     "SteepleChase" },
    { +00.0000000, -00.0000000 ,  -1     ,""          ,     "" }
};

