#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{
  void BezierCurve::evaluateStep()
  {
    // TODO Part 1.
    // Perform one step of the Bezier curve's evaluation at t using de Casteljau's algorithm for subdivision.
    // Store all of the intermediate control points into the 2D vector evaluatedLevels.
    //(1-t)p1 * p2t

    int s =  evaluatedLevels.size();
    if (s < numControlPoints) {
      std::vector<Vector2D> v2 = std::vector<Vector2D>();
      for (int x = 0; x < numControlPoints - s; x++) {
       //printf("%d\n", x);
       Vector2D p1 = evaluatedLevels[s - 1][x];
       //printf("%d\n", x);
       Vector2D p2 = evaluatedLevels[s - 1][x+1];
       //printf("%d\n", x);
       Vector2D v = Vector2D((1-t)*p1 + p2*t);
       //printf("%d\n", x);
       v2.push_back(v);
       //printf("%d\n", x);
      } 
      evaluatedLevels.push_back(v2);
    }
    //controlPoints
    //evaluatedLevels
    //numControlPoints
    //t
  }


  Vector3D BezierPatch::evaluate(double u, double v) const
  {
    // TODO Part 2.
    // Evaluate the Bezier surface at parameters (u, v) through 2D de Casteljau subdivision.
    // (i.e. Unlike Part 1 where we performed one subdivision level per call to evaluateStep, this function
    // should apply de Casteljau's algorithm until it computes the final, evaluated point on the surface)
    std::vector<Vector3D> v3 = std::vector<Vector3D>();
    for (int i = 0; i < 4; i++) {
      std::vector<Vector3D> v2 = std::vector<Vector3D>();
      for (int j = 0; j < 4; j++) {
        v2.push_back(controlPoints[i][j]);
      }
      v3.push_back(evaluate1D(v2, u));
    }
    return evaluate1D(v3, v);
  }

  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> points, double t) const
  {
    // TODO Part 2.
    // Optional helper function that you might find useful to implement as an abstraction when implementing BezierPatch::evaluate.
    // Given an array of 4 points that lie on a single curve, evaluates the Bezier curve at parameter t using 1D de Casteljau subdivision.
    int s = points.size();
    while (s > 1) {
      std::vector<Vector3D> v2 = std::vector<Vector3D>();
      for (int x = 0; x < s - 1; x++) {
       //printf("%d\n", x);
       Vector3D p1 = points[x];
       //printf("%d\n", x);
       Vector3D p2 = points[x+1];
       //printf("%d\n", x);
       Vector3D v = Vector3D((1-t)*p1 + p2*t);
       //printf("%d\n", x);
       v2.push_back(v);
       //printf("%d\n", x);
      } 
      points = v2;
      s = points.size();
    }


    return points[0];
  }



  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // TODO Returns an approximate unit normal at this vertex, computed by
    // TODO taking the area-weighted average of the normals of neighboring
    // TODO triangles, then normalizing.
    Vector3D n(0,0,0); // initialize a vector to store your normal sum
    HalfedgeCIter h = halfedge(); // Since we're in a Vertex, this returns a halfedge
                                 // pointing _away_ from that vertex


    do
    {
       // check if the current halfedge is on the boundary
       HalfedgeCIter h_orig = h;
       Vector3D p1 = h->vertex()->position;
       Vector3D p2 = h->next()->vertex()->position;
       Vector3D p3 = h->next()->next()->vertex()->position;
       // do
       // {

       //  h = h->next();
       // }
       // while (h != h_orig);
       n += cross(p2-p1, p3-p1);


       // move to the next halfedge around the vertex
       h = h_orig->twin()->next();
    }
    while( h != halfedge() );

    return n.unit();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // TODO This method should flip the given edge and return an iterator to the flipped edge.
    if (e0->halfedge()->isBoundary()) {
      return e0;
    }

    HalfedgeIter h = e0->halfedge();
    HalfedgeIter h2 = h->twin();
    HalfedgeIter a = h->next();
    HalfedgeIter b = h->next()->next();
    HalfedgeIter c = h->twin()->next()->next();
    HalfedgeIter d = h->twin()->next();
    Vector3D p1 = h->next()->next()->vertex()->position;
    Vector3D p2 = h->twin()->next()->next()->vertex()->position;
    h->face()->halfedge() = h;
    h2->face()->halfedge() = h2;
    h->setNeighbors( c,
                            h->twin(),
                            b->vertex(),
                            h->edge(),
                            h->face() );
    h2->setNeighbors( b,
                            h2->twin(),
                            c->vertex(),
                            h2->edge(),
                            h2->face() );
    a->setNeighbors( h,
                            a->twin(),
                            a->vertex(),
                            a->edge(),
                            h->face() );
    b->setNeighbors( d,
                            b->twin(),
                            b->vertex(),
                            b->edge(),
                            h2->face() );
    c->setNeighbors( a,
                            c->twin(),
                            c->vertex(),
                            c->edge(),
                            h->face() );
    d->setNeighbors( h2,
                            d->twin(),
                            d->vertex(),
                            d->edge(),
                            h2->face() );
    //h->_vertex = b->vertex();
    //h2->_vertex = c->vertex();


    return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // TODO This method should split the given edge and return an iterator to the newly inserted vertex.
    // TODO The halfedge of this vertex should point along the edge that was split, rather than the new edges.

    if (e0->halfedge()->isBoundary()) {
      return e0->halfedge()->vertex();
    }

    HalfedgeIter h = e0->halfedge();
    HalfedgeIter h2 = h->twin();
    HalfedgeIter a = h->next();
    HalfedgeIter b = h->next()->next();
    HalfedgeIter c = h->twin()->next()->next();
    HalfedgeIter d = h->twin()->next();
    Vector3D p1 = h->next()->next()->vertex()->position;
    Vector3D p2 = h->twin()->next()->next()->vertex()->position;
    Vector3D p3 = 0.5*p1 + 0.5*p2;
    VertexIter m = newVertex();
    m->position = p3;
    HalfedgeIter mA = newHalfedge();
    HalfedgeIter Bm = newHalfedge();
    HalfedgeIter mB = newHalfedge();
    HalfedgeIter Dm = newHalfedge();
    HalfedgeIter mD = newHalfedge();
    HalfedgeIter Cm = newHalfedge();
    HalfedgeIter mC = newHalfedge();
    HalfedgeIter Am = newHalfedge();
    EdgeIter eA = newEdge();
    EdgeIter eB = newEdge();
    EdgeIter eC = newEdge();
    EdgeIter eD = newEdge();
    b->face() = newFace();
    d->face() = newFace();
    mA->setNeighbors( a,
                            Am,
                            m,
                            eA,
                            a->face() );
    Bm->setNeighbors( mA,
                            mB,
                            b->vertex(),
                            eB,
                            a->face() );
    mB->setNeighbors( b,
                            Bm,
                            m,
                            eB,
                            b->face() );
    Dm->setNeighbors( mB,
                            mD,
                            d->vertex(),
                            eD,
                            b->face() );
    mD->setNeighbors( d,
                            Dm,
                            m,
                            eD,
                            d->face() );
    Cm->setNeighbors( mD,
                            mC,
                            c->vertex(),
                            eC,
                            d->face() );
    mC->setNeighbors( c,
                            Cm,
                            m,
                            eC,
                            c->face() );
    Am->setNeighbors( mC,
                            mA,
                            a->vertex(),
                            eA,
                            c->face() );
    a->next() = Bm;
    b->next() = Dm;
    c->next() = Am;
    d->next() = Cm;
    a->face()->halfedge() = a;
    b->face()->halfedge() = b;
    c->face()->halfedge() = c;
    d->face()->halfedge() = d;
    m->halfedge() = mA;
    eA->halfedge() = mA;
    eB->halfedge() = mB;
    eC->halfedge() = mC;
    eD->halfedge() = mD;
    deleteEdge(h->edge());
    deleteHalfedge(h);
    deleteHalfedge(h2);

    return m;
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // Each vertex and edge of the original surface can be associated with a vertex in the new (subdivided) surface.
    // Therefore, our strategy for computing the subdivided vertex locations is to *first* compute the new positions
    // using the connectity of the original (coarse) mesh; navigating this mesh will be much easier than navigating
    // the new subdivided (fine) mesh, which has more elements to traverse. We will then assign vertex positions in
    // the new mesh based on the values we computed for the original mesh.


    // TODO Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // TODO and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // TODO a vertex of the original mesh.


    // TODO Next, compute the updated vertex positions associated with edges, and store it in Edge::newPosition.


    // TODO Next, we're going to split every edge in the mesh, in any order.  For future
    // TODO reference, we're also going to store some information about which subdivided
    // TODO edges come from splitting an edge in the original mesh, and which edges are new,
    // TODO by setting the flat Edge::isNew.  Note that in this loop, we only want to iterate
    // TODO over edges of the original mesh---otherwise, we'll end up splitting edges that we
    // TODO just split (and the loop will never end!)


    // TODO Now flip any new edge that connects an old and new vertex.


    // TODO Finally, copy the new vertex positions into final Vertex::position.
  }

}
