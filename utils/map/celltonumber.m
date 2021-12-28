function A = celltonumber(segment,point)
mx = segment(1);
my = segment(2);
A = point(1) + (point(2)-1)*mx + (point(3)-1)*mx*my;
end
