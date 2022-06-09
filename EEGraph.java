import java.util.*;
import java.io.*;

public class EEGraph
{
  static int n;
  static int delta;
  static Digraph D;

  public static void main(String [] args) throws IOException
  {
    Scanner scan = new Scanner(System.in);
    /*System.out.print("n = ");
    n = scan.nextInt();
    System.out.print("delta = ");
    int delta = scan.nextInt();
    System.out.print("alpha = ");
    double alpha = scan.nextDouble();
    System.out.print("beta = ");
    double beta = scan.nextDouble();
    System.out.print("r0 = ");
    double r0 = scan.nextDouble();
    */
    n=100; int r_0=64;
    D = new Digraph(n);

    double [] parameters = findParameters(r_0);
    double alpha = parameters[0];
    double beta = parameters[1];
    System.out.println("alpha = " + alpha + ". beta = " + beta + ". delta = " + delta);
    baseExpander(n, alpha, beta, r_0);

    //algorithm 5.1
    Random rand = new Random();
    for(int v = 1; v < n-1; v++)
    {
      for(int i = 1; i < Math.ceil(delta * Math.log(v+1)/Math.log(2)); i++)
      {
        int k = (int)Math.floor(rand.nextDouble(Math.log(v+1)));
        Edge e = new Edge(v - (int)Math.pow(2,k),v);
        if(D.addEdge(e))
          System.out.println("added " + e.toString());
      }
    }

    System.out.println("Longest path = " + findLongestPath());
    System.out.println("erdosAttack");
    ArrayList<Integer> removedVertices = erdosAttack(.2);
    //for(int v : removedVertices)
    //  System.out.println("Removed " + v);
  }

  public static void printGraph()
  {
    for(int i  = 0; i < n;i++)
    {
      if(D.adj[i]!=null)
        for(Edge e : D.adj(i))
          System.out.println(e.toString());
    }
  }

  //for now we assume beta = 1-alpha
  static double[] findParameters(int r_0)
  {
    double alpha = .01;
    double parameters[]=new double[2];
    delta = n;
    while(alpha < .5-1/r_0)
    {
      //System.out.println("in loop. alpha="+alpha);
      int currDelta = (int)Math.ceil(-2*(-alpha*Math.log(alpha)-(1-alpha)*Math.log(1-alpha))*Math.log(2)/(alpha*Math.log((2-alpha)/2)));
      if(currDelta<delta && 2*Math.PI*r_0 >=(1-alpha)/Math.pow(alpha,3))
      {
        delta = currDelta;
        parameters[0]=alpha;
        parameters[1]=1-alpha;
      }
      alpha+=.001;
    }
    return parameters;
  }

  public static void baseExpander(int n, double a, double b, double r0)
  {
    for(int i = 1 ; i<n-1;i++)
      for(int j = 1 ; j<n-1;j++)
        if(j-i>0 && j-i<2*r0)
          D.addEdge(new Edge(i,j));
  }

  @SuppressWarnings("unchecked")
  public static ArrayList<Integer> erdosAttack(double e)
  {
    double p = findP(e);
    int r = (int)(Math.log(n)/p);
    System.out.println("r  = " + r + ". p = " + p);
    ArrayList<Edge>[] C = (ArrayList<Edge>[]) new ArrayList[r];
    for(int k = 0; k < r; k++)
    {
      C[k] = new ArrayList<Edge>();
      for(int u = 1; u < n-1; u++)
      {
        for(Edge s : D.adj(u))
        {
          int length = s.destination-u;
          if(length >= Math.pow(2,k*p) && length < Math.pow(2,(k+1)*p))
          {
            System.out.println("added " + s.toString() + " to C["+k+"]: " + C[k].add(s));
          }
        }
      }
    }
    ArrayList<Edge> C_k = new ArrayList<Edge> ();
    int k = 0;
    for(int i = 0; i< r; i++)
    {
      if(C[i].size()<= D.m/r && C[i].size()!=0)
      {
        C_k = C[i];
        k = i;
        break;
      }
    }

    /*
    for(int i = 0; i<r;i++)
    {
      System.out.println("Size of C_"+i+" = " + C[i].size());
    }

    System.out.println("edges in C_k: ");
    for(Edge s: C_k)
    {
      System.out.print(s.toString() + " ");
    }
    */

    int bigConst = (int)(Math.pow(2,k*p)*(1+Math.pow(2,p/2)));
    int numVSets = (int) n/bigConst;
    ArrayList<Integer>[] B = (ArrayList<Integer>[]) new ArrayList[numVSets];
    for(int i = 0; i< numVSets; i++)
    {
      B[i] = new ArrayList<Integer>();
      for(int v = 0; v<n-1; v++)
      {
        if(v >= i*bigConst && v < (i+1)*bigConst)
        {
          B[i].add(v);
        }
      }
    }

    ArrayList<Integer>[] H = (ArrayList<Integer>[]) new ArrayList[numVSets];
    for(int i  = 0; i< numVSets; i++)
    {
      H[i] = new ArrayList<Integer>();
      for(int v : B[i])
      {
        if(i*bigConst <= v && i*bigConst + Math.pow(2,k*p) >v)
        {
          H[i].add(v);
        }
      }
    }

    ArrayList<Integer> S = new ArrayList<Integer>();
    for(Edge s : C_k)
    {
      if(!S.contains(s.destination)) S.add(s.destination);
    }
    for(ArrayList<Integer> H_i : H)
    {
      for(int v : H_i)
      {
        if(!S.contains(v))
        {
          S.add(v);
        }
      }
    }

    for(int v: S)
    {
      deleteVertex(v);
    }

    System.out.println("Longest path = " + findLongestPath());
    return S;//return set of vertices removed
  }

  public static double findP(double e)
  {
    e*=n; //note that e represents the total number of vertices to remove here, but represents
          //the fraction to remove in erdosAttack()
    //int lo = 0;
    //int hi = (int)Math.log(n);
    double  p = 0;
    while(true)//while(lo<=hi)
    {
      //int p = (lo+hi)/2;
      double test = (D.m*p*(1+Math.pow(2,p/2))+n*Math.log(n))/(Math.log(n)*(1+Math.pow(2,p/2)))-e*Math.log(n);
      System.out.println("test = " + test + ". p = " + p);
      if(test>=0)
        return p;
      p+=.01;
      //lo = p+1;//if condition is not satisfied, ``guess'' for p was too low(since test is montonically increasing)
    }
    //return -1;
  }

  public static void deleteVertex(int v)
  {
    /*
    if(v > n)
    {
      throw new RuntimeException("Vertex " + v + " not in digragh);
    }
    */
    ArrayList<Edge> toRemove = new ArrayList<Edge>(); //don't want to mess with iterable, while iterating

    for(int i = 1; i<n-1;i++)
    {
      Edge e = new Edge(i,v);
      if(D.adj[i]!=null && D.adj[i].contains(e) && i!=v)
      {
        //System.out.println("removing edge " + i + v );
        D.adj[i].remove(e);
      }
    }
    for(Edge r : D.adj(v))
    {
      toRemove.add(r); //don't want to mess with iterable, while iterating
    }
    D.adj[v].removeAll(toRemove); //don't want to mess with iterable, while iterating
    n--;
  }

   // Function that returns the longest path
   static int findLongestPath()
   {
     ArrayList<Edge> longestPath = new ArrayList<Edge>();
     int longestLength = 0;
     for(int source = 1; source<n-1; source++)
     {
       for(int dest = 0; dest<n;dest++)
       {
         int currLength = 0;
         ArrayList<Edge> currPath = new ArrayList<Edge>();
         D.dijkstras(source,dest);

         if(!D.marked[dest])//if no path from source to dest, skip
          continue;

         Stack<Integer> path = new Stack<Integer>();
         for (int x = dest; x != source; x = D.edgeTo[x])
         {
            path.push(x);
            currLength++;
            currPath.add(new Edge(x, D.edgeTo[x]));
         }

         int prevVertex = source;
         while(!path.empty())
         {
           int w = path.pop();
           prevVertex = w;
         }

         if(currLength>longestLength)
         {
          longestLength = currLength;
          longestPath = currPath;
         }
       }
     }

     System.out.print("longest path: ");
     for(Edge e : longestPath) System.out.print(e.toString() + " ");
     System.out.println();

     return longestLength;
   }

}//end EEGraph class------------------------------------------------------------

class Digraph
{
  public final int v;
  public int m;
  public LinkedList<Edge>[] adj;
  public boolean marked[];
  public int distTo[];
  public int edgeTo[];


  public Digraph(int v)
  {
    if (v < 0) throw new RuntimeException("Number of vertices must be nonnegative");
    this.v = v;
    this.m = 0;
    @SuppressWarnings("unchecked")
    LinkedList<Edge>[] temp =
    (LinkedList<Edge>[]) new LinkedList[2*v];
    adj = temp;
    for (int i = 1; i < 2*v; i++)
    adj[i] = new LinkedList<Edge>();
  }

  public boolean addEdge(Edge edge)
  {
    int from = edge.source;
    int to = edge.destination;

    //case where both are null
    if(adj[from] == null && adj[to] == null)
    {
      adj[from] = new LinkedList<Edge>();
      adj[from].add(edge);
      m++;
      return true;
    }

    //case where to is null, and from does not contain edge
    if(adj[from] != null && !adj[to].contains(edge) && adj[to] == null )
    {
      adj[from].add(edge);
      m++;
      return true;
    }

    //case where from is null, and to does not contain edge
    if(adj[from] == null && adj[to] != null && !adj[to].contains(new Edge(to, from)))
    {
      adj[from] = new LinkedList<Edge>();
      adj[from].add(edge);
      m++;
      return true;
    }

    //case where neither is null, and neither contains edge
    if(adj[from] != null && !adj[from].contains(edge) && adj[to] != null && !adj[to].contains(new Edge(to, from)))
    {
      adj[from].add(edge);
      m++;
      return true;
    }

    return false; //otherwise

  }

  public Iterable<Edge> adj(int v)
  {
    return adj[v];
  }

  public void dijkstras(int source, int destination)
  {
    marked = new boolean[this.v];
    distTo = new int[this.v];
    edgeTo = new int[this.v];


    for (int i = 0; i < v; i++)
    {
      distTo[i] = -1; //initialize distance to every vertex to be -1
      marked[i] = false; //and unmarked
    }

    distTo[source] = 0; //source has distance of 0, and is marked
    marked[source] = true;
    int nMarked = 1; //number of marked vertices is 1

    int current = source;
    while (nMarked < this.v) //while there are unmarked vertices
    {
      for (Edge w : adj(current)) //check every adjacent edge to source
      {
        if (distTo[current]+1 > distTo[w.destination]) //if the adding this edge increases the distance to the a vertex v,
        {
          distTo[w.destination] = distTo[current]+1; //update the distTo array for that vertex
          edgeTo[w.destination] = current; //and update the edgeTo array to note the longestPath
        }
      }

      int max = -1;
      current = -1;

      //I should do this more efficiently with a priority queue
      for(int i=0; i<distTo.length; i++)
      {
        if(marked[i])
          continue;
        if(distTo[i] > max)
        {
          max = distTo[i];
          current = i;
        }
      }

      //TODO: Update marked[] and nMarked. Check for disconnected graph.
      if(current==-1) break;
      marked[current]=true;
      nMarked--;

    }
  }
}//end Digraph class------------------------------------------------------------

class Edge
{
  public int source;
  public int destination;

  public Edge(int source, int destination)
  {
    this.source = source;
    this.destination = destination;
  }
  @Override
  public String toString()
  {
    return source + "->" + destination;
  }

  public boolean equals(Object other)
  {
    if (other instanceof Edge)
    {
      Edge otherEdge = (Edge) other;
      return source == otherEdge.source && destination == otherEdge.destination;
    }
    return false;
  }
}//end Edge class---------------------------------------------------------------
