#ifndef COORDINATE_HPP
#define COORDINATE_HPP

#include <iostream>
#include <sstream>
#include <iomanip>
#ifdef _WIN32
#define M_PI       3.14159265358979323846
#endif
#include <cmath>

namespace utils {
namespace target {

enum {
    cSwMilsPerCircle    = 2048,
};

///
/// \brief The tSwMil class
/// 目标角位信息
class tSwMil {
public:
    static inline uint16_t Add(uint16_t mil1, uint16_t mil2) {
        return (mil1+mil2)%cSwMilsPerCircle;
    }
    static inline uint16_t Sub(uint16_t mil1, uint16_t mil2) {
        return mil1>=mil2 ? mil1-mil2 : cSwMilsPerCircle+mil1-mil2;
    }
    static inline uint16_t DegreeToMil(double deg) {
        return cSwMilsPerCircle*deg/360.0+0.5;
    }
    //将密位转换成角度
    static inline double MilToDegree(uint16_t mil) {
        return mil*360.0/(double)cSwMilsPerCircle;
    }
    //将密位转换成弧度
    static inline double MilToRadian(uint16_t mil) {
        return 2*M_PI*mil/(double)cSwMilsPerCircle;
    }

    static double Rad(double deg) {
        return deg*M_PI/180.0;
    }
    static double Deg(double rad) {
        return rad*180.0/M_PI;
    }
    static double TanToDegree(double x, double y) {
        double angle = M_PI/2-atan2(y, x);
        if(angle<0) angle+=M_PI*2;
        return 360.0*angle/(2*M_PI);
    }
};

///
/// \brief The tCartCoorPoint struct
/// 直角坐标点
struct tCartCoorPoint {
    double X_m = 0;
    double Y_m = 0;
    tCartCoorPoint() {}
    tCartCoorPoint(double x, double y) { Set(x, y); }
    void Set(double x, double y) { X_m = x; Y_m = y; }
    double DistanceTo(const tCartCoorPoint& pnt2) {
        return std::sqrt((X_m-pnt2.X_m)*(X_m-pnt2.X_m) + (Y_m-pnt2.Y_m)*(Y_m-pnt2.Y_m));
    }
    double DistanceSquareTo(const tCartCoorPoint& pnt2) {
        return (X_m-pnt2.X_m)*(X_m-pnt2.X_m) + (Y_m-pnt2.Y_m)*(Y_m-pnt2.Y_m);
    }
    tCartCoorPoint operator-(const tCartCoorPoint& other) const {
        return tCartCoorPoint(X_m - other.X_m, Y_m - other.Y_m);
    }
    tCartCoorPoint operator+(const tCartCoorPoint& other) const {
        return tCartCoorPoint(X_m + other.X_m, Y_m + other.Y_m);
    }
    friend tCartCoorPoint operator/(const tCartCoorPoint& obj, int divisor) {
        if (divisor != 0) {
            return tCartCoorPoint(obj.X_m/divisor, obj.Y_m/divisor);
        } else {
            return obj;
        }
    }
    bool operator!=(const tCartCoorPoint& other) const {
        return X_m != other.X_m || Y_m != other.Y_m;
    }
};

///
/// \brief The tPolarCoorPoint struct
/// 极坐标点
struct tPolarCoorPoint {
public:
    tPolarCoorPoint() {}
    tPolarCoorPoint(double bearing_deg, double distance_m) : m_bearing_deg(bearing_deg), m_distance_m(distance_m) {
        m_bearing_mil = tSwMil::DegreeToMil(m_bearing_deg);
    }
    tPolarCoorPoint(uint16_t bearing_swMil, uint32_t distance_m) { Set(bearing_swMil, distance_m); }
    void Set(uint16_t bearing_mil, uint32_t distance_m) {
        m_bearing_mil = bearing_mil;
        m_bearing_deg = tSwMil::MilToDegree(m_bearing_mil);
        m_distance_m = distance_m;
    }
    uint16_t BearingMil() const { return m_bearing_mil; }
    double BearingDegree() const { return m_bearing_deg; }
    double Distance_m() const { return m_distance_m; }

private:
    uint16_t m_bearing_mil = 0;
    double m_bearing_deg = 0;
    double m_distance_m = 0;
};

///
/// \brief The tTargetSharpPnt struct
/// 轮廓坐标点
struct tTargetSharpPnt {
    // 轮廓点相对于雷达(本船)的极坐标
    tPolarCoorPoint relPolarPntToRadar;
    tPolarCoorPoint absPolarPntToRadar;
    // 轮廓点相对于目标中心点的直角坐标
    tCartCoorPoint relCartPntToTarget;
    tCartCoorPoint absCartPntToTarget;
    // 目标轮廓长宽信息
    uint16_t width_m;
    uint16_t length_m;
};

const double EARTH_RADIUS = 6371000.0; // 地球半径，单位：米

///
/// \brief The tGeoCoorPoint class
/// 大地坐标点（经纬度）
class tGeoCoorPoint {
public:
    tGeoCoorPoint() {}
    tGeoCoorPoint(double lng, double lat) : longitude(lng), latitude(lat) {}
    virtual ~tGeoCoorPoint() {}
public:
    double Lng() const { return longitude; }
    double Lat() const { return latitude; }
    bool operator!=(const tGeoCoorPoint& other) const {
        return longitude != other.Lng() || latitude != other.Lat();
    }
    std::pair<std::string, std::string> ToDmmSting() {
        return std::make_pair(convertToDmm(longitude, 3), convertToDmm(latitude, 2));
    }
    std::string convertToDmm(double lng_lat, int deg_digits) {
        // 取整数部分作为度数部分
        int degrees = static_cast<int>(lng_lat);

        // 计算度数的小数部分
        double fraction = std::abs(lng_lat - degrees);

        // 将小数部分转换为分数部分
        double minutes = fraction * 60;

        // 将度数和分数部分格式化为dddmm.mmmmm格式
        std::stringstream ss;
        ss << std::setfill('0') << std::setw(deg_digits) << degrees; // 3位度
        ss << std::setw(2) << static_cast<int>(minutes); // 分钟部分
        ss << "." << std::setw(5) << static_cast<int>((minutes - static_cast<int>(minutes))*100000+0.5); // 小数部分，保留5位小数
        return ss.str();
    }
    //计算同一纬度下，两个经度之间的东西方向距离
    static double LngDistanceInSameLat(double baseLng, double lng, double baseLat) {
        return (2*M_PI*EARTH_RADIUS*cos(tSwMil::Rad(baseLat))) / 360.0 * (lng-baseLng);
    }
    //计算同一经度下，两个纬度之间的南北方向距离
    static double LatDistanceInSameLng(double baseLat, double lat) {
        return (2*M_PI*EARTH_RADIUS) / 360.0 * (lat-baseLat);
    }
private:
    double longitude = 0; // 经度
    double latitude = 0; // 纬度

};

///
/// \brief The tCoordinateConverter class
/// 坐标转换
class tCoordinateConverter {
public:
    // 将直角坐标转换为极坐标
    static std::pair<double, double> CartToPolar(double x, double y) {
        double r = sqrt(x*x + y*y);
        double theta = tSwMil::TanToDegree(x, y);
        return std::make_pair(r, theta);
    }
    static std::pair<double, double> CartToPolar(std::pair<double, double> cart) {
        return CartToPolar(cart.first, cart.second);
    }

    static tPolarCoorPoint CartToPolar(const tCartCoorPoint& cartPnt) {
        std::pair<double, double> p = tCoordinateConverter::CartToPolar(cartPnt.X_m, cartPnt.Y_m);
        return tPolarCoorPoint(p.second, p.first);
    }
    // 将极坐标转换为直角坐标
    static tCartCoorPoint PolarToCart(const tPolarCoorPoint& polarPnt) {
        double theta = tSwMil::MilToRadian(polarPnt.BearingMil());
        return tCartCoorPoint(polarPnt.Distance_m()*sin(theta), polarPnt.Distance_m()*cos(theta));
    }
    // 极坐标转换为大地坐标
    static tGeoCoorPoint PolarToGeo(const tGeoCoorPoint& basePnt, const tPolarCoorPoint& polarPnt) {
        double distance = polarPnt.Distance_m();
        double bearing = polarPnt.BearingDegree();
        double longitude = basePnt.Lng();
        double latitude = basePnt.Lat();
        double radian_bearing = tSwMil::Rad(bearing);
        double radian_longitude = tSwMil::Rad(longitude);
        double radian_latitude = tSwMil::Rad(latitude);
        double new_latitude = asin(sin(radian_latitude) * cos(distance / EARTH_RADIUS) +
                                   cos(radian_latitude) * sin(distance / EARTH_RADIUS) * cos(radian_bearing));
        double new_longitude = radian_longitude + atan2(sin(radian_bearing) * sin(distance / EARTH_RADIUS) * cos(radian_latitude),
                                                        cos(distance / EARTH_RADIUS) - sin(radian_latitude) * sin(new_latitude));
        return tGeoCoorPoint(tSwMil::Deg(new_longitude), tSwMil::Deg(new_latitude));
    }
    // 大地坐标转换为直角坐标
    static tCartCoorPoint GeoToCart(double baseLng, double baseLat, double lng, double lat) {
        return tCartCoorPoint(tGeoCoorPoint::LngDistanceInSameLat(baseLng, lng, baseLat), tGeoCoorPoint::LatDistanceInSameLng(baseLat, lat));
    }
    static tCartCoorPoint GeoToCart(const tGeoCoorPoint& baseGeo, const tGeoCoorPoint& geo) {
        return tCartCoorPoint(tGeoCoorPoint::LngDistanceInSameLat(baseGeo.Lng(), geo.Lng(), baseGeo.Lat()),
                              tGeoCoorPoint::LatDistanceInSameLng(baseGeo.Lat(), geo.Lat()));
    }

};

///
/// \brief The tSpeed struct
/// 目标速度信息
struct tSpeed {
    //极坐标
    double speed;
    double course;
    //直角坐标
    double speedX;
    double speedY;
    //根据直角坐标计算极坐标
    inline void SetXY(double speed_X, double speed_Y) {
        speedX = speed_X;
        speedY = speed_Y;
        std::pair<double, double> p = tCoordinateConverter::CartToPolar(speedX, speedY);
        speed = p.first;
        course = p.second;
    }
    //根据极坐标计算直角坐标
    inline void SetSC(double speed_S, double course_C) {
        speed = speed_S;
        course = course_C;
        speedX = speed_S*sin(course*M_PI/180.0);
        speedY = speed_S*cos(course*M_PI/180.0);
    }
    //计算速度矢量差
    inline tSpeed operator-(const tSpeed& other) const {
        tSpeed result;
        result.SetXY(speedX - other.speedX, speedY - other.speedY);
        return result;
    }
};

} //namespace target
} //namespace utils

#endif // COORDINATE_HPP
