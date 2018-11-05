function run_swtest(x)

for alpha = 0.05:0.05:0.95
    res = swtest(x, alpha);
    fprintf("Alpha: %f Swtest: %d\n", alpha, res);
    
end
