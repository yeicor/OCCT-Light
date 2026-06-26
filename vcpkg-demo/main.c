#include <occtl/occtl_core.h>
#include <occtl/occtl_topo.h>
#include <occtl/occtl_prim_solid.h>
#include <stdio.h>
#include <stdlib.h>

int main(void)
{
  uint32_t ver_major, ver_minor, ver_patch;
  occtl_runtime_version(&ver_major, &ver_minor, &ver_patch);
  printf("OCCT-Light %u.%u.%u (ABI %u)\n",
         ver_major, ver_minor, ver_patch,
         occtl_runtime_abi_version());
  printf("OCCT version: %s\n", occtl_runtime_occt_version());

  occtl_graph_t* graph = NULL;
  occtl_status_t st;

  st = occtl_graph_create(&graph);
  if (st != OCCTL_OK) { fprintf(stderr, "occtl_graph_create failed\n"); return 1; }

  occtl_prim_box_info_t box_info = OCCTL_PRIM_BOX_INFO_INIT;
  box_info.dx = 10.0;
  box_info.dy = 20.0;
  box_info.dz = 30.0;

  occtl_node_id_t solid = OCCTL_NODE_ID_INVALID;
  st = occtl_prim_make_box(graph, &box_info, &solid);
  if (st != OCCTL_OK) { fprintf(stderr, "occtl_prim_make_box failed\n"); occtl_graph_free(graph); return 1; }

  size_t cnt;
  occtl_graph_face_count(graph, &cnt);    printf("  faces:  %zu\n", cnt);
  occtl_graph_edge_count(graph, &cnt);    printf("  edges:  %zu\n", cnt);
  occtl_graph_vertex_count(graph, &cnt);  printf("  vertices: %zu\n", cnt);
  occtl_graph_shell_count(graph, &cnt);   printf("  shells: %zu\n", cnt);
  occtl_graph_solid_count(graph, &cnt);   printf("  solids: %zu\n", cnt);

  occtl_graph_free(graph);
  printf("OK\n");
  return 0;
}
