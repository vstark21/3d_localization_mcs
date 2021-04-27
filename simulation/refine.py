def loss_function(lines, p):
    L = 0
    for li in lines:
        L += li.perp_dist(p)
    return L

def refine(lines, eps=0.000001, lr=0.05):
    p_init = [0, 0, 0]
    prev_loss = float('inf')
    c = 0
    num_iter=0
    while True:
        L = loss_function(lines, p_init)

        L2 = loss_function(lines, [p_init[0]+eps, p_init[1], p_init[2]])
        dLx = (L2 - L) / eps

        L2 = loss_function(lines, [p_init[0], p_init[1]+eps, p_init[2]])
        dLy = (L2 - L) / eps

        L2 = loss_function(lines, [p_init[0], p_init[1], p_init[2]+eps])
        dLz = (L2 - L) / eps
        
        p_init = [p_init[0] - lr*dLx, p_init[1] - lr*dLy, p_init[2] - lr*dLz]

        if prev_loss < L:
            lr *= 0.1
            c += 1
        if c == 3 or num_iter == 15000:
            break
        prev_loss = L
        num_iter+=1

    print(num_iter, " NUMITER")
    print(L)

    return p_init

        

