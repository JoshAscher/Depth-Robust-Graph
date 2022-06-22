import java.util.*;
import java.io.*;

public class EEGraph
{
  static int n;
  static int delta;
  static double e;
  static double d;
  static Digraph D;

  public static void main(String [] args) throws IOException
  {
    Scanner scan = new Scanner(System.in);

    //for now, all parameters are hardcoded
    /*System.out.print("n = ");
    n = scan.nextInt();
    System.out.print("r0 = ");
    double r0 = scan.nextDouble();
    System.out.println("Desired depth robustness? e  = ");
    e = scan.nextDouble();
    System.out.print("d = ");
    d = scan.nextDouble();
    */

    n=1000; int r_0=32; e = .11; d = .22;
    D = new Digraph(n);
    double alpha = findParameters(e,d,r_0);
    double beta = 1-alpha;//simplification for now;

    delta = (int)Math.ceil(-2*(-alpha*Math.log(alpha)-(1-alpha)*Math.log(1-alpha))*Math.log(2)/(alpha*Math.log((2-alpha)/2))); //condition for delta outlined in Overleaf doc

    System.out.println("alpha = " + alpha + ". beta = " + beta + ". delta = " + delta);
    baseExpander(n, alpha, beta, r_0); //construct base of graph

    //algorithm 5.1
    Random rand = new Random();
    for(int v = 1; v < n-1; v++)
    {
      for(int i = 1; i < Math.ceil(delta * Math.log(v+1)/Math.log(2)); i++)
      {
        int k = (int)Math.floor(rand.nextDouble(Math.log(v+1)));
        Edge e = new Edge(v - (int)Math.pow(2,k),v);
        if(D.addEdge(e))
          continue;//System.out.println("added " + e.toString());
      }
    }

    //System.out.println("Longest path = " + findLongestPath());
    ArrayList<Integer> removedVertices = erdosAttack(.11);
    //for(int v : removedVertices)
      //System.out.println("Removed " + v);
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
  static double findParameters(double e, double d, int r_0)
  {
    double alpha  = 0;
    double currAlpha = 0;
    while(currAlpha<.5)
    {
      double e_max = (1-2*currAlpha)/(1+2*currAlpha);
      double d_max = 1-e/e_max;
      if(e < e_max && d<=d_max)
      {
        alpha = currAlpha;
      }
      currAlpha+=.001;
    }
    return alpha;
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
    int edgeCount = 0;
    int r = 7;
    double p = Math.log(n)/r;
    System.out.println("r  = " + r + ". p = " + p);

    //divide edges into sets based on their length
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
            C[k].add(s);
            edgeCount++;
          }
        }
      }
    }

    System.out.println("total edges in the C_i = " + edgeCount + ". true size = "+D.m);

    //find a set with number of edges below average to remove
    ArrayList<Edge> C_k = new ArrayList<Edge> ();
    int k = 0;
    for(int i = 0; i< r; i++)
    {
      if(C[i].size()<= D.m/r && C[i].size()!=0)
      {
        C_k = C[i];
        k = i;
        System.out.println("changed k to " + k);
        break;
      }
    }

    //print out size of each C_i(debugging)
    for(int i  = 0; i<r; i++)
      System.out.println("Size of C_" + i + " = " + C[i].size());

    //construct blocks of consecutive vertices
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

    //split each block into a head and tail
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

    //remove the end vertex of each in the chosen C_k
    ArrayList<Integer> S = new ArrayList<Integer>();
    for(Edge s : C_k)
    {
      if(!S.contains(s.destination)) S.add(s.destination);
    }

    //remove every vertex in the head of each block
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

    //int longPath = findLongestPath();
    //System.out.println("Longest path = " + longPath);
    //System.out.println("This graph is (" +e*n + ", " + longPath +")-depth robust");
    System.out.println("total number of vertices removed = " + S.size());
    Collections.sort(S); //want to print them out in order
    return S;//return set of vertices removed
  }

  public static void deleteVertex(int v) //throws Exception
  {
    if(v > n)
    {
      throw new RuntimeException("Vertex " + v +  "not in digragh");
    }
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
