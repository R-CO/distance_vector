# Distance Vector

## Step-by-Step

- c(x,v) = cost for direct link from x to v
Node x maintains costs of direct links c(x,v)

- Dx(y) = estimate of least cost from x to y
Node x maintains distance vector Dx = [Dx(y): y є N ]

- Node x maintains its neighbors’ distance vectors
For each neighbor v, x maintains Dv = [Dv(y): y є N ]

- Each node v periodically sends Dv to its neighbors
And neighbors(x) update their own distance vectors
Dx(y) ← minv{c(x,v) + Dv(y)} for each node y ε N
