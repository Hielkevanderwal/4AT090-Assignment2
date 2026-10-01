function ang = wrapToHalfPi(ang_rad)
     tmp = mod(ang_rad + pi/2,pi);
     ang = tmp + pi*(ang_rad>0 & tmp==0) - pi/2;
end

