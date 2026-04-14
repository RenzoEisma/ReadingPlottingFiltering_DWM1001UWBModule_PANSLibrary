function success = process_uwb(x, y, z)
    fprintf('MATLAB received: X: %.2f, Y: %.2f, Z: %.2f\n', x, y, z);
    success = true;
end