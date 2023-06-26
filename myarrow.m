function [a] = myarrow(x,y)



ax = gca;
axpos = get(ax, 'Position');
X = get(gca,'XLim');
Y = get(gca,'YLim');

% ensure 0 based axes
x = x - X(1);
y = y - Y(1);

difX = X(2) - X(1);
difY = Y(2) - Y(1);
newx = x./difX;
newy = y./difY;

finalX = [newx(1)*axpos(3)+axpos(1) newx(2)*axpos(3)+axpos(1)];
finalY = [newy(1)*axpos(4)+axpos(2) newy(2)*axpos(4)+axpos(2)];
a = annotation('arrow',finalX,finalY);

end