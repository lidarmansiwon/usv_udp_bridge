#include <string>

struct NavigationMsg {
  uint32_t usv_id;
  float x, y, psi, u, v, r, w;
};

struct ControlMsg {
  uint32_t usv_id;
  float x_d, y_d, psi_d, u_d, r_d;
};
