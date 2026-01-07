struct Pos2D {
    float x;
    float y;
    float theta;
};

struct Landmark {
    int id;
    float x;
    float y;
};

struct Control {
    float v;      // linear velocity
    float w;      // angular velocity
    float dt;     // time step
};
struct Observation {
    int landmark_id;
    float range;
    float bearing;
};