# Distance Vector

## description

### Distance Vector (DV)

#### At each node, x

D_x(y) = minimum over all v { c(x,v) + D_v(y) }
The cost from a node x to a node y is the cost from x to a directly connected node v plus the cost to get from v to y.
This is the minimum cost considering both the cost from x to v and the cost from v to y.

#### At each node x

- INITIALISATION:
   for all destinations y in N:
      D_x(y) = c(x,y) /* If y not a neighbour, c(x,y) = Infinity */
   for each neighbour w
      D_w(y) = Infinity for all destinations y in N
   for each neighbour w
      send distance vector D_x = [D_x(y): y in N] to w

- LOOP
   wait (until I see a link cost change to some neighbour w or until
      I receive a distance vector from some neighbour w)
   for each y in N:
      D_x(y) = min_v{c(x,v) + D_v(y)}

   if D_x(y) changed for any destination y
      send distance vector D_x = [D_x(y): y in N] to all neighbours.

- FOREVER

Note: Infinity is a number sufficiently large that no legal cost is greater than or equal to infinity.
The value of infinity is left for you to choose.

### Poisoned Reverse (PR)

In Poisoned Reverse, if a node A routes through another node B to get to a destination C, then A will advertise to B that its distance to C is Infinity.
A will continue to advertise this to B as long as A uses B to get to C. This will prevent B from using A to get to C if B's own connection to C fails.

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
