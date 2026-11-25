function [x_hat, e_t] = kalman_update_st(y, x_pred, K, ss)
  
    e_t = y - ss.C * x_pred;      
    x_hat = x_pred + K * e_t;

end