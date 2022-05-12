import java.util.*;
import java.io.*;

public class EEGraph
{
  static int n;
  static Digraph D;

  public static void main(String [] args) throws IOException
  {
    Scanner scan = new Scanner(System.in);
    System.out.print("n = ");
    n = scan.nextInt();
    System.out.print("delta = ");
    int delta = scan.nextInt();
    System.out.print("alpha = ");
    double alpha = scan.nextDouble();
    System.out.print("beta = ");
    double beta = scan.nextDouble();
    System.out.print("r0 = ");
    double r0 = scan.nextDouble();
    D = new Digraph(n);
    baseExpander(n, alpha, beta, r0);
    printGraph();

    Random rand = new Random();
    for(int v = 1; v < n-1; v++)
    {
      for(int i = 1; i < Math.ceil(delta * Math.log(v+1)/Math.log(2)); i++)
      {
        int k = (int)Math.floor(rand.nextDouble(Math.log(v+1)));
        Edge e = new Edge(v,v - (int)Math.pow(2,k));
        if(D.addEdge(e))
          System.out.println("added " + e.toString());
      }
    }

    printGraph();
    //erdosAttack(1,2);
    //System.out.println("erdosAttack");
    //printGraph();
  }

  public static void printGraph()
  {
    for(int i  = 1; i < n-1;i++)
    {
      if(D.adj[i]!=null)
        for(Edge e : D.adj(i))
          System.out.println(e.toString());
    }
  }

  public static void baseExpander(int n, double a, double b, double r0)
  {
    for(int i = 1 ; i<n-1;i++)
    {
      for(int j = 1 ; j<n-1;j++)
        if(j-i>0 && j-i<2*r0)
          D.addEdge(new Edge(i,j));
    }
  }

  @SuppressWarnings("unchecked")
  public static ArrayList<Integer> erdosAttack(int e, int d)
  {
    int p = (int)(-2*Math.log(d));
    int r = (int)(Math.log(n)/p);
    ArrayList<Edge>[] C = (ArrayList<Edge>[]) new ArrayList[r];
    for(int k = 0; k < r; k++)
    {
      for(int u = 1; u < n-1; u++)
      {
        for(Edge s : D.adj(u))
        {
          int length = s.destination-u;
          if(length >= Math.pow(2,k*p) && length < Math.pow(2,(k+1)*p))
          {
            C[k].add(s);
          }
        }
      }
    }
    ArrayList<Edge> C_k = new ArrayList<Edge> ();
    int K = 0;
    for(int i = 0; i< r; i++)
    {
      if(C[i].size()>= D.m/r)
      {
        C_k = C[i];
        K = i;
        break;
      }
    }

    int bigConst = (int)(Math.pow(2,K*p)*(1+Math.pow(2,p/2)));
    int numVSets = (int) n/bigConst;
    ArrayList<Integer>[] B = (ArrayList<Integer>[]) new ArrayList[numVSets];
    for(int i = 0; i< numVSets; i++)
    {
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
      for(int v : B[i])
      {
        if(i*bigConst <= v && i*bigConst + Math.pow(2,K*p) >v)
        {
          H[i].add(v);
        }
      }
    }
    ArrayList<Integer> S = new ArrayList<Integer>();
    for(Edge s : C_k)
    {
      S.add(s.destination);
    }
    for(ArrayList<Integer> H_i : H)
    {
      S.addAll(H_i);
    }
    for(int v: S)
    {
      deleteVertex(v);
    }
    return S;//return set of vertices removed
  }

  public static void deleteVertex(int v)
  {
    if(v > n)
    {
      throw new RuntimeException("Vertex " + v + " not in digraph");
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
    n--;
  }
}//end EEGraph class------------------------------------------------------------

class Digraph
{
  public final int v;
  public int m;
  public LinkedList<Edge>[] adj;


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
    if(adj[from].contains(edge))
      return false;
    adj[from].add(edge);
    m++;
    return true;
  }

  public Iterable<Edge> adj(int v)
  {
    return adj[v];
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
