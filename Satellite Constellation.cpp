#include <iostream>
#include <limits>
#include <cmath>

const int DEPTH = 20;
const float INF = std::numeric_limits<float>::infinity();

const float MU = 3.986E14;
const float EARTH_RADIUS = 6.371E6;
const float LIGHT_SPEED = 299792458;

class Vector {
public:
    Vector(float _x = 0, float _y = 0, float _z = 0) {
        x = _x;
        y = _y;
        z = _z;
    }

    float getMagnitude() {
        return sqrt(dot(*this));
    }

    Vector normalize() {
        return (*this) / getMagnitude();
    }

    float dot(Vector other) {
        return x * other.getX() + y * other.getY() + z * other.getZ();
    }

    Vector cross(Vector other) {
        float xc = y * other.getZ() - z * other.getY();
        float yc = z * other.getX() - x * other.getZ();
        float zc = x * other.getY() - y * other.getX();
        return Vector(xc, yc, zc);
    }

    float getX() {
        return x;
    }

    float getY() {
        return y;
    }

    float getZ() {
        return z;
    }

    void setX(float _x) {
        x = _x;
    }

    void setY(float _y) {
        y = _y;
    }

    void setZ(float _z) {
        z = _z;
    }

    bool operator==(Vector other) {
        return x == other.getX() && y == other.getY() && z == other.getZ();
    }

    Vector operator+(Vector other) {
        return Vector(x + other.getX(), y + other.getY(), z + other.getZ());
    }

    Vector operator-(Vector other) {
        return Vector(x - other.getX(), y - other.getY(), z - other.getZ());
    }

    Vector operator*(float a) {
        return Vector(x * a, y * a, z * a);
    }

    Vector operator/(float a) {
        return Vector(x / a, y / a, z / a);
    }

private:
    float x, y, z;
};

class Satellite {
public:
    Satellite(int _id = 0, float _sma = 0, float _ecc = 0, float _inc = 0, float _raan = 0, float _aop = 0, float _ta = 0) {
        id = _id;
        sma = _sma;
        ecc = _ecc;
        inc = _inc;
        raan = _raan;
        aop = _aop;
        ta = _ta;

        position = Vector(0, 0, 0);
    }

    Satellite (Vector pos) {
        position = pos;
    }

    Vector getPosition() {
        if (position.getMagnitude() > 0) return position;

        float ea = 2 * atan(tan(ta / 2) * sqrt((1 - ecc) / (1 + ecc)));
        float rMag = sma * (1 - ecc * cos(ea));

        float x = rMag * (cos(raan) * cos(aop + ta) - sin(raan) * sin(aop + ta) * cos(inc));
        float y = rMag * (sin(raan) * cos(aop + ta) + cos(raan) * sin(aop + ta) * cos(inc));
        float z = rMag * (sin(inc) * sin(aop + ta));

        position = Vector(x, y, z);

        return position;
    }

    bool isTargetVisible(Vector target) {
        Vector pos = getPosition();
        Vector dir = target - pos;
        float t = -pos.dot(dir) / dir.dot(dir);

        if (0 < t && t < 1) return (pos + dir * t).getMagnitude() >= EARTH_RADIUS;

        return true;
    }

    bool isTargetVisible(Satellite target) {
        return isTargetVisible(target.getPosition());
    }

    float getDistance(Vector target) {
        return (getPosition() - target).getMagnitude();
    }

    float getDistance(Satellite target) {
        return getDistance(target.getPosition());
    }

    void advanceTimeSecs(float s) {
        Vector pos = Vector(0, 0, 0);

        float T = 2 * M_PI * sqrt(sma * sma * sma / MU);

        float E0 = 2 * atan(tan(ta / 2) * sqrt((1 - ecc) / (1 + ecc)));
        float M0 = E0 - ecc * sin(E0);

        float M = M0 + 2 * M_PI * s / T;
        float E = M;
        for (int i = 0; i < 3; i++) {
            E -= (E - ecc * sin(E) - M) / (1 - ecc * cos(E));
        }
        ta = 2 * atan(tan(E / 2) * sqrt((1 + ecc) / (1 - ecc)));
    }

    int getId(){
        return id;
    }

    bool operator==(Satellite other) {
        return id == other.getId();
    }

    bool operator!=(Satellite other) {
        return id != other.getId();
    }

private:
    int id;

    float sma;
    float ecc;
    float inc;
    float raan;
    float aop;
    float ta;

    Vector position;
};

class Constellation {
public:
    Constellation(int _satCount, Satellite* _satellite) {
        satCount = _satCount;
        satellites = _satellite;
    }

    void advanceTimeSecs(float s) {
        for (int i = 0; i < satCount; i++) {
            satellites[i].advanceTimeSecs(s);
        }
    }

    void sendMessage(float lat1, float lon1, float lat2, float lon2) {
        lat1 *= M_PI / 180;
        lon1 *= M_PI / 180;
        lat2 *= M_PI / 180;
        lon2 *= M_PI / 180;
        float x1 = EARTH_RADIUS * cos(lat1) * cos(lon1);
        float y1 = EARTH_RADIUS * cos(lat1) * sin(lon1);
        float z1 = EARTH_RADIUS * sin(lat1);
        Vector origin(x1, y1, z1);
        float x2 = EARTH_RADIUS * cos(lat2) * cos(lon2);
        float y2 = EARTH_RADIUS * cos(lat2) * sin(lon2);
        float z2 = EARTH_RADIUS * sin(lat2);
        Vector target(x2, y2, z2);

        Satellite startingPoint(origin);
        int route[DEPTH] = { 0 };
        int shortestRoute[DEPTH] = { 0 };
        float shortestLength = INF;
        findRecepient(startingPoint, target, 0, route, 0, shortestRoute, &shortestLength);

        if (shortestLength == INF) {
            std::cout << "Target is unreachable";
            return;
        }

        std::cout << "The shortest route takes " << shortestLength / LIGHT_SPEED << " seconds" << std::endl;
        std::cout << "It takes path: ";
        for (int i = 0; i < DEPTH; i++) {
            if (shortestRoute[i] == 0) break;
            std::cout << shortestRoute[i] << " ";
        }
    }

private:
    int satCount;
    Satellite* satellites;

    void findRecepient(Satellite current, Vector target, int step,
                       int* route, float distance, int* shortestRoute, float* shortestLength) {
        if (step > DEPTH) return;

        if (current.isTargetVisible(target)) {
            distance += current.getDistance(target);
            if (distance < *shortestLength) {
                for (int i = 0; i < DEPTH; i++) {
                    shortestRoute[i] = route[i];
                }
                *shortestLength = distance;
            }
            return;
        }

        for (int i = 0; i < satCount; i++) {

            if (satellites[i] == current) continue;

            bool isInRoute = false;
            for (int j = 0; j < DEPTH; j++) {
                if (route[j] == satellites[i].getId()) {
                    isInRoute = true;
                    break;
                }
            }

            if (isInRoute) continue;

            if (step > 0 && satellites[i].getDistance(target) > current.getDistance(target)) continue;

            if (!current.isTargetVisible(satellites[i])) continue;

            route[step] = satellites[i].getId();
            float d = current.getDistance(satellites[i]);
            findRecepient(satellites[i], target, step + 1, route, distance + d, shortestRoute, shortestLength);
            route[step] = 0;
        }
    }
};

int main() {
    int n = 25;
    Satellite* net = new Satellite[n];
    for (int i = 0; i < n; i++) {
        net[i] = Satellite(i + 1, EARTH_RADIUS * 1.1, 0, M_PI / 4, 2 * M_PI / n * i, 0, 2 * M_PI / n * i);
    }

    Constellation cons(n, net);
    cons.advanceTimeSecs(1E4);
    cons.sendMessage(60, 0, -40, 180);

    return 0;
}
