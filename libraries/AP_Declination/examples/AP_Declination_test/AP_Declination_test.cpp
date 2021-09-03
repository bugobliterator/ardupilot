// #include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
// #include <AP_Param/AP_Param.h>
// #include <StorageManager/StorageManager.h>
// #include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Declination/AP_Declination.h>
// #include <Filter/Filter.h>
#include <stdio.h>

// get the point at some distance and bearing from a given point
// see http://www.movable-type.co.uk/scripts/latlong.html
// δ = d/R
// φ2 = asin( sin φ1 ⋅ cos δ + cos φ1 ⋅ sin δ ⋅ cos θ )
// λ2 = λ1 + atan2( sin θ ⋅ sin δ ⋅ cos φ1, cos δ − sin φ1 ⋅ sin φ2 )

void get_new_pos(const struct Location &prev_loc, float bearing, float distance, struct Location &ret)
{
    bearing = radians(wrap_360(bearing));
    double r = distance / RADIUS_OF_EARTH;
    double prev_lat = radians(prev_loc.lat*1.0e-7);
    double prev_lng = radians(prev_loc.lng*1.0e-7);
    ret.lat = degrees(asin((sin(prev_lat)*cos(r)) +
                    (cos(prev_lat)*sin(r)*cos(bearing))))*1.0e7;
    double ret_lat = radians(ret.lat*1.0e-7);
    ret.lng = degrees(prev_lng + atan2(sin(bearing)*sin(r)*cos(prev_lat),
                           cos(r)-(sin(prev_lat)*sin(ret_lat))))*1.0e7;
    // printf("Dist: %f BRNG: %f Old Pos:%f %f New Pos:%f %f\n", distance, degrees(bearing), prev_loc.lat*1.0e-7f, prev_loc.lng*1.0e-7f, ret.lat*1.0e-7f, ret.lng*1.0e-7f);
}

// Get points of a square with side length 2*radius centered at the given location
void get_square(const struct Location &loc, float radius, int32_t &lat_max, int32_t &lat_min, int32_t &lng_max, int32_t &lng_min)
{
    struct Location tmp_loc;
    // Top left corner
    get_new_pos(loc, 45, radius, tmp_loc);
    lat_min = tmp_loc.lat;
    // Top right corner
    get_new_pos(loc, 315, radius, tmp_loc);
    lng_max = tmp_loc.lng;
    // Bottom right corner
    get_new_pos(loc, 225, radius, tmp_loc);
    lat_max = tmp_loc.lat;
    // Bottom left corner
    get_new_pos(loc, 135, radius, tmp_loc);
    lng_min = tmp_loc.lng;
}

int main(int argc, char **argv)
{
    // Extract command line arguments (lat, lon, radius, declination, granularity)
    if (argc != 6) {
        printf("Usage: %s lat lon radius declination granularity\n", argv[0]);
        return 1;
    }
    float lat = atof(argv[1]);
    float lon = atof(argv[2]);
    float radius = atof(argv[3])*1000.0;
    float search_decl_rad = atof(argv[4]);
    float search_decl = degrees(search_decl_rad);
    float granularity = atof(argv[5]);
    struct Location loc(lat*1.0e7, lon*1.0e7, 0, Location::AltFrame::ABOVE_HOME);
    
    int32_t start_search_lat, end_search_lat, start_search_long, end_search_long;
    // Get points of a square with side length 2*radius centered at the given location
    get_square(loc, radius, start_search_lat, end_search_lat, start_search_long, end_search_long);
    
    printf("Beginning Search: Lat Min: %f, Lat Max: %f, Lng Min: %f, Lng Max: %f\n", (double)start_search_lat*1.0e-7, (double)end_search_lat*1.0e-7, (double)start_search_long*1.0e-7, (double)end_search_long*1.0e-7);

    struct Location search_loc(start_search_lat, start_search_long, 0, Location::AltFrame::ABOVE_HOME);    

    // Search through the location
    while (search_loc.lat <= end_search_lat) {
        search_loc.lng = start_search_long;
        while (search_loc.lng <= end_search_long) {
            const float declination = AP_Declination::get_declination(search_loc.lat*1.0e-7f, search_loc.lng*1.0e-7f);
            if (is_equal(declination, search_decl)) {
                printf("%f, %f\n", search_loc.lat*1.0e-7f, search_loc.lng*1.0e-7f);
            }
            search_loc.lng = Location::wrap_longitude(search_loc.lng + (granularity*1.0e7));
        }
        search_loc.lat = Location::limit_lattitude(search_loc.lat + (granularity*1.0e7));
    }
    printf("Finished.\n\n");
}
