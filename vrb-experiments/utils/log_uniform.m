function val = log_uniform(min_val, max_val)
    val = exp(rand() * log(max_val / min_val)) * min_val;
end