function x_pred = kalman_predict_linear(x_hat, u, ss)
    
    x_pred = ss.A * x_hat + ss.B * u;

end