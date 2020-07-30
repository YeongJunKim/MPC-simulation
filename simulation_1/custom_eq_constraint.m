function r = custom_eq_constraint(X,U,data)

r = X(end,:) - data.References(end,:);
r = sum(r);