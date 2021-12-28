function cellcoordinate = numbertocell(map,number)
m = map.segment ; 
mx = m(1);
my = m(2);
nz = ceil(number/(mx*my));
ny = ceil((number - (nz-1)*mx*my)/mx);
nx = number - (ny-1)*mx - (nz-1)*mx*my;
cellcoordinate = [nx ny nz];
end