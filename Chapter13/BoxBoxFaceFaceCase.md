### Question

What happens when there is a face-face collision in fact? how would algorithm output the contact point refer to [collide_fine.cpp](https://github.com/idmillington/cyclone-physics/blob/d75c8d9edeebfdc0deebe203fe862299084b1e30/src/collide_fine.cpp#L409)?

```cpp
unsigned CollisionDetector::boxAndBox(
    const CollisionBox &one,
    const CollisionBox &two,
    CollisionData *data
    )
{
    //if (!IntersectionTests::boxAndBox(one, two)) return 0;

    // Find the vector between the two centres
    Vector3 toCentre = two.getAxis(3) - one.getAxis(3);

    // We start assuming there is no contact
    real pen = REAL_MAX;
    unsigned best = 0xffffff;

    // Now we check each axes, returning if it gives us
    // a separating axis, and keeping track of the axis with
    // the smallest penetration otherwise.
    CHECK_OVERLAP(one.getAxis(0), 0);
    CHECK_OVERLAP(one.getAxis(1), 1);
    CHECK_OVERLAP(one.getAxis(2), 2);

    CHECK_OVERLAP(two.getAxis(0), 3);
    CHECK_OVERLAP(two.getAxis(1), 4);
    CHECK_OVERLAP(two.getAxis(2), 5);

    // Store the best axis-major, in case we run into almost
    // parallel edge collisions later
    unsigned bestSingleAxis = best;

    CHECK_OVERLAP(one.getAxis(0) % two.getAxis(0), 6);
    CHECK_OVERLAP(one.getAxis(0) % two.getAxis(1), 7);
    CHECK_OVERLAP(one.getAxis(0) % two.getAxis(2), 8);
    CHECK_OVERLAP(one.getAxis(1) % two.getAxis(0), 9);
    CHECK_OVERLAP(one.getAxis(1) % two.getAxis(1), 10);
    CHECK_OVERLAP(one.getAxis(1) % two.getAxis(2), 11);
    CHECK_OVERLAP(one.getAxis(2) % two.getAxis(0), 12);
    CHECK_OVERLAP(one.getAxis(2) % two.getAxis(1), 13);
    CHECK_OVERLAP(one.getAxis(2) % two.getAxis(2), 14);

    // Make sure we've got a result.
    assert(best != 0xffffff);

    // We now know there's a collision, and we know which
    // of the axes gave the smallest penetration. We now
    // can deal with it in different ways depending on
    // the case.
    if (best < 3)
    {
        // We've got a vertex of box two on a face of box one.
        fillPointFaceBoxBox(one, two, toCentre, data, best, pen);
        data->addContacts(1);
        return 1;
    }
    else if (best < 6)
    {
        // We've got a vertex of box one on a face of box two.
        // We use the same algorithm as above, but swap around
        // one and two (and therefore also the vector between their
        // centres).
        fillPointFaceBoxBox(two, one, toCentre*-1.0f, data, best-3, pen);
        data->addContacts(1);
        return 1;
    }
    else
    {
        // We've got an edge-edge contact. Find out which axes
        best -= 6;
        unsigned oneAxisIndex = best / 3;
        unsigned twoAxisIndex = best % 3;
        Vector3 oneAxis = one.getAxis(oneAxisIndex);
        Vector3 twoAxis = two.getAxis(twoAxisIndex);
        Vector3 axis = oneAxis % twoAxis;
        axis.normalise();

        // The axis should point from box one to box two.
        if (axis * toCentre > 0) axis = axis * -1.0f;

        // We have the axes, but not the edges: each axis has 4 edges parallel
        // to it, we need to find which of the 4 for each object. We do
        // that by finding the point in the centre of the edge. We know
        // its component in the direction of the box's collision axis is zero
        // (its a mid-point) and we determine which of the extremes in each
        // of the other axes is closest.
        Vector3 ptOnOneEdge = one.halfSize;
        Vector3 ptOnTwoEdge = two.halfSize;
        for (unsigned i = 0; i < 3; i++)
        {
            if (i == oneAxisIndex) ptOnOneEdge[i] = 0;
            else if (one.getAxis(i) * axis > 0) ptOnOneEdge[i] = -ptOnOneEdge[i];

            if (i == twoAxisIndex) ptOnTwoEdge[i] = 0;
            else if (two.getAxis(i) * axis < 0) ptOnTwoEdge[i] = -ptOnTwoEdge[i];
        }

        // Move them into world coordinates (they are already oriented
        // correctly, since they have been derived from the axes).
        ptOnOneEdge = one.transform * ptOnOneEdge;
        ptOnTwoEdge = two.transform * ptOnTwoEdge;

        // So we have a point and a direction for the colliding edges.
        // We need to find out point of closest approach of the two
        // line-segments.
        Vector3 vertex = contactPoint(
            ptOnOneEdge, oneAxis, one.halfSize[oneAxisIndex],
            ptOnTwoEdge, twoAxis, two.halfSize[twoAxisIndex],
            bestSingleAxis > 2
            );

        // We can fill the contact.
        Contact* contact = data->contacts;

        contact->penetration = pen;
        contact->contactNormal = axis;
        contact->contactPoint = vertex;
        contact->setBodyData(one.body, two.body,
            data->friction, data->restitution);
        data->addContacts(1);
        return 1;
    }
    return 0;
}
#undef CHECK_OVERLAP

```







Short answer: this code *never* has a special “face–face” branch.
 Even if the true geometric situation is face–face, the algorithm will **approximate it as either:**

- a **point–face** (vertex–face) contact, or
- an **edge–edge** contact that sometimes degenerates to edge–face.

Let’s walk through what actually happens.

------

## 1. How the algorithm classifies the contact

Recall the `best` index:

```
// after checking 15 axes:
if (best < 3)                 // 0,1,2
    // vertex of box TWO on face of box ONE
else if (best < 6)            // 3,4,5
    // vertex of box ONE on face of box TWO
else                          // 6..14
    // edge–edge case
```

There is **no `face–face` case** here. SAT tells us which axis gave the **smallest penetration**; this axis is always interpreted as:

- a **face normal** → “point–face” contact
- a **cross product of edges** → “edge–edge” contact

A perfect face–face configuration (two parallel faces pressed together) still has a face normal as its “best axis”, so it falls into the **point–face branch**.

------

## 2. What happens in a face–face situation?

Imagine two boxes with perfectly parallel faces touching, e.g.:

- Box A: face with normal +X
- Box B: opposite face with normal –X, pressed into A

SAT finds that one of the axes (say `one.getAxis(0)` or `two.getAxis(0)`) has the smallest penetration → so `best` is in `[0..5]`.

Say we end up with `best < 3`, so we enter:

```
fillPointFaceBoxBox(one, two, toCentre, data, best, pen);
```

This is the “vertex of TWO on face of ONE” case.

### 2.1 How `fillPointFaceBoxBox` picks the contact normal

```
Vector3 normal = one.getAxis(best);
if (one.getAxis(best) * toCentre > 0)
{
    normal = normal * -1.0f;
}
```

- `one.getAxis(best)` is the face normal of box ONE.
- It’s flipped if needed so that `normal` points **from box ONE toward box TWO**.
- In face–face, this is the correct contact normal (the face normal).

### 2.2 How it chooses the contact “point”

This is the key part:

```
Vector3 vertex = two.halfSize;
if (two.getAxis(0) * normal < 0) vertex.x = -vertex.x;
if (two.getAxis(1) * normal < 0) vertex.y = -vertex.y;
if (two.getAxis(2) * normal < 0) vertex.z = -vertex.z;

contact->contactPoint = two.getTransform() * vertex;
```

What’s going on?

- `two.halfSize` is `( +hx, +hy, +hz )` in TWO’s local space = one corner.
- For each local axis of box TWO, we check `two.getAxis(i) * normal`:
  - If the local axis points **against** the collision normal, we flip the sign of that coordinate.
- This is exactly how you compute the **support point of box TWO in direction `-normal`**.
  - i.e. the vertex of box TWO that is **furthest along –normal**, or
  - from the opposite perspective, the vertex of TWO **most deeply penetrating into ONE**.

Then we transform that vertex into world space and use it as the contact point.

> So:
>  **Even in a perfect face–face contact**, the algorithm picks **just one vertex** of one box that lies on that face and uses that as the single contact point.

Geometrically, the true contact region is a **whole rectangle** (face–face overlap), but this algorithm compresses it into **one representative point + a normal + penetration depth**.

So the output is:

- `contactNormal`: the face normal (pointing from ONE to TWO)
- `penetration`: the minimal overlap depth `pen`
- `contactPoint`: **one corner** of the overlapping face from box TWO

------

## 3. What about the “edge–face” mention inside `contactPoint()`?

In the edge–edge branch (`best >= 6`), we do:

```
Vector3 vertex = contactPoint(
    ptOnOneEdge, oneAxis, one.halfSize[oneAxisIndex],
    ptOnTwoEdge, twoAxis, two.halfSize[twoAxisIndex],
    bestSingleAxis > 2
);
```

Inside `contactPoint`:

```
// If either of the edges has the nearest point out of bounds, 
// then the edges aren't crossed, we have an edge-face contact. 
// Our point is on the edge, which we know from the useOne parameter.
if (mua > oneSize || mua < -oneSize ||
    mub > twoSize || mub < -twoSize)
{
    return useOne ? pOne : pTwo;
}
```

This is handling a **degenerate case** for the “edge–edge” path:

- If the closest points of the infinite lines lie outside the segment extents,
   then the geometry isn’t really “edge–edge crossing” but more like **edge–face**.
- Then it just returns **one of the endpoints** (`pOne` or `pTwo`) as the contact point.

Again: still ends up as **one point**.

------

## 4. So, how does this algorithm treat true face–face contact?

**In practice:**

- It does **not** produce a special face–face manifold.
- It **always** outputs **exactly one contact**:
  - Either:
    - point–face (most face–face cases land here), or
    - edge–edge (sometimes degenerating to edge–face via `contactPoint`).
- For a true face–face overlap, it:
  - picks the face normal as `contactNormal`
  - picks the **deepest vertex** of one box (in direction of the normal) as `contactPoint`
  - uses the SAT penetration amount `pen`

Physics resolution then uses this single point+normal+penetration to push boxes apart and compute impulses.

**Modern engines** often build a **contact manifold** (2–4 points) for face–face, via clipping polygons against the reference face, so the constraint is more stable. Millington’s sample code is a simplified teaching version: one contact only, to keep the math and implementation short.