// Public-library OpenVSP 3.55.0 geometry regression; all four cases required.
// No CLI argument parser, flow solve, private-access macros or source replacement.
// Native ReadFile bootstraps one tiny quad, its two explicit alternate triangles
// and one two-node TE chain; UpdateMeshes is the method under test.
#include "VSP_Geom.H"
#include <array>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

using VSPAERO_SOLVER::VSP_GEOM;
using VSPAERO_SOLVER::VSP_NODE;
using Vec = std::array<double,3>;

// Static storage zero-initializes upstream fields not explicitly set by init(),
// matching the native CLI's static owner. No layout tricks or private writes.
static VSP_GEOM geometries[4];

struct Attachment {
    int sheet, trail, te, wake, fine_te, fine_wake;
    Vec expected;
};

struct Result {
    const char* name;
    int level, levels, input_vertices, reported_surface_count, actual_surface_nodes;
    bool unused, nonidentity, surfaces_unchanged, other_fine_nodes_unchanged;
    bool roots_attached, repeated_update_idempotent, all_coordinates_finite;
    bool deflect_wake_matches_fine, undeflect_wake_matches_fine;
    bool deflect_fine_nodes_unchanged, undeflect_fine_nodes_unchanged;
    int checked_wake_nodes;
    std::vector<Attachment> attachment;
    bool passed() const {
        return surfaces_unchanged && other_fine_nodes_unchanged && roots_attached &&
               repeated_update_idempotent && all_coordinates_finite &&
               deflect_wake_matches_fine && undeflect_wake_matches_fine &&
               deflect_fine_nodes_unchanged && undeflect_fine_nodes_unchanged;
    }
};

static Vec xyz(VSP_NODE& node) { return {node.x(),node.y(),node.z()}; }
static void put(VSP_NODE& node, const Vec& p) { node.x()=p[0];node.y()=p[1];node.z()=p[2]; }
static bool finite(const Vec& p) { return std::isfinite(p[0])&&std::isfinite(p[1])&&std::isfinite(p[2]); }

static void require(bool value,const char* message) {
    if(!value) throw std::runtime_error(message);
}

static int fine_node(VSP_GEOM& geometry,int level,int node) {
    for(int current=level;current>0;--current) {
        require(node>0&&node<=geometry.Grid(current).NumberOfNodes(),"invalid coarse node index");
        node=geometry.Grid(current).NodeList(node).FineGridNode();
    }
    require(node>0&&node<=geometry.Grid(0).NumberOfNodes(),"invalid finest node index");
    return node;
}

static void write_mesh(const std::filesystem::path& stem,bool unused) {
    const int d=unused?1:0;
    std::ofstream mesh(stem.string()+".vspgeom");
    require(bool(mesh),"cannot create fixture mesh");
    mesh<<"# vspgeom v3\n1\n"<<4+d<<" 1 1\n";
    // The optional unreferenced interior point changes indexing only. Its
    // position remains inside the existing rectangle bounding box.
    if(unused)mesh<<"0.5 0.5 0\n";
    mesh<<"0 0 0\n1 0 0\n1 1 0\n0 1 0\n1\n";
    mesh<<"4 "<<1+d<<' '<<2+d<<' '<<3+d<<' '<<4+d<<"\n";
    mesh<<"1 1 0 0 1 0 1 1 0 1\n1 1\n";
    mesh<<"1\n2 1 "<<2+d<<' '<<3+d<<"\n";
    mesh<<"1 2 "<<1+d<<' '<<2+d<<' '<<3+d<<' '<<1+d<<' '<<3+d<<' '<<4+d<<"\n";
    mesh<<"1 1 1 0 0 1 0 1 1 0 0 1 1 0 1\n";
    mesh.close();require(bool(mesh),"fixture mesh write failed");
    std::ofstream key(stem.string()+".vkey");
    key<<"# VSPGEOM v3 Tag Key File\n"<<stem.string()<<".vspgeom\n1\n\n";
    key<<"# part#,geom#,surf#,gname,gid,thick,plate,copy#,geomcopy#\n";
    key<<"1,0,0,public_quad,public_quad,0,3,1,1\n\n1\n\n";
    key<<"# tag#,part#,ssname1,ssname2,...,ssid1,ssid2,...\n1,1\n";
    key.close();require(bool(key),"fixture key write failed");
}

static Result run_case(VSP_GEOM& geometry,const std::filesystem::path& directory,
                       const char* name,int level,bool unused) {
    const auto stem=directory/name;
    write_mesh(stem,unused);
    geometry.SolveOnMGLevel()=level;
    geometry.SetNumberOfWakeTrailingNodes(4);
    std::string input=stem.string();std::vector<char> native_name(input.begin(),input.end());native_name.push_back('\0');
    geometry.ReadFile(native_name.data());
    require(geometry.NumberOfGridLevels()>=level,"fixture did not create requested multigrid level");
    require(geometry.NumberOfVortexSheets()==1,"fixture must produce exactly one sheet");
    auto& grid=geometry.Grid(0);
    std::set<int> surface_nodes;
    for(int i=1;i<=grid.NumberOfLoops();++i) {
        auto& loop=grid.LoopList(i);
        if(loop.SurfaceID()>0)for(int j=1;j<=loop.NumberOfNodes();++j)surface_nodes.insert(loop.Node(j));
    }
    require(surface_nodes.size()==4,"fixture must retain four actual physical surface nodes");
    // Translate the physical surface via its actual connectivity. The native
    // surface count is NOT assumed to be a contiguous finest-node prefix.
    for(int node:surface_nodes) {
        auto p=xyz(grid.NodeList(node));p[0]+=.125;p[1]-=.25;p[2]+=.375;put(grid.NodeList(node),p);
    }
    Result result{name,level,geometry.NumberOfGridLevels(),4+(unused?1:0),
                  grid.NumberOfSurfaceNodes(),static_cast<int>(surface_nodes.size()),unused,false,true,true,true,true,true,
                  true,true,true,true,0,{}};
    std::set<int> expected_root_nodes;
    for(int k=1;k<=geometry.NumberOfVortexSheets();++k) {
        auto& sheet=geometry.VortexSheet(k);
        require(!sheet.IsARotor(),"fixture must not contain rotor updates");
        for(int i=1;i<=sheet.NumberOfTrailingVortices();++i) {
            auto& trail=sheet.TrailingVortex(i);
            const int te=trail.TE_Node(),wake=trail.GlobalNode(1);
            const int ft=fine_node(geometry,level,te),fw=fine_node(geometry,level,wake);
            require(surface_nodes.count(ft)==1,"TE must map to actual surface connectivity");
            require(surface_nodes.count(fw)==0,"wake root must map outside physical surface connectivity");
            require(expected_root_nodes.insert(fw).second,"wake roots must map to distinct finest nodes");
            result.nonidentity=result.nonidentity||(ft!=te)||(fw!=wake);
            result.attachment.push_back({k,i,te,wake,ft,fw,xyz(grid.NodeList(ft))});
        }
    }
    require(result.attachment.size()==2,"fixture must have two distinct TE root attachments");
    if(level==0)require(!result.nonidentity,"zero-level control must be identity");
    if(level==1&&!unused)require(!result.nonidentity,"identity control unexpectedly compacted indices");
    if(level>0&&unused)require(result.nonidentity,"nonidentity control did not compact the unused vertex");
    std::vector<Vec> before(grid.NumberOfNodes()+1);
    for(int i=1;i<=grid.NumberOfNodes();++i)before[i]=xyz(grid.NodeList(i));
    geometry.UpdateMeshes();
    for(int i=1;i<=grid.NumberOfNodes();++i) {
        const auto p=xyz(grid.NodeList(i));result.all_coordinates_finite=result.all_coordinates_finite&&finite(p);
        if(surface_nodes.count(i))result.surfaces_unchanged=result.surfaces_unchanged&&(p==before[i]);
        if(!expected_root_nodes.count(i))result.other_fine_nodes_unchanged=result.other_fine_nodes_unchanged&&(p==before[i]);
    }
    for(const auto& a:result.attachment)result.roots_attached=result.roots_attached&&(xyz(grid.NodeList(a.fine_wake))==a.expected);
    std::vector<Vec> once(grid.NumberOfNodes()+1);
    for(int i=1;i<=grid.NumberOfNodes();++i)once[i]=xyz(grid.NodeList(i));
    geometry.UpdateMeshes();
    for(int i=1;i<=grid.NumberOfNodes();++i)result.repeated_update_idempotent=result.repeated_update_idempotent&&(xyz(grid.NodeList(i))==once[i]);
    require(geometry.NumberOfControlSurfaces()==0,"fixture must exercise both public zero-control paths");
    const auto check_wake = [&]() {
        bool matches=true;
        int checked=0;
        for(int k=1;k<=geometry.NumberOfVortexSheets();++k) {
            auto& sheet=geometry.VortexSheet(k);
            for(int i=1;i<=sheet.NumberOfTrailingVortices();++i) {
                auto& trail=sheet.TrailingVortex(i);
                for(int j=1;j<=trail.NumberOfNodes();++j) {
                    const int finest=fine_node(geometry,level,trail.GlobalNode(j));
                    const Vec actual{trail.WakeNodeX(j),trail.WakeNodeY(j),trail.WakeNodeZ(j)};
                    matches=matches&&finite(actual)&&(actual==xyz(grid.NodeList(finest)));
                    ++checked;
                }
            }
        }
        require(checked==8,"fixture must check every node on both four-node trailing vortices");
        result.checked_wake_nodes=checked;
        return matches;
    };
    const auto fine_unchanged = [&]() {
        bool same=true;
        for(int i=1;i<=grid.NumberOfNodes();++i) {
            const auto p=xyz(grid.NodeList(i));
            same=same&&(p==once[i]);
            result.all_coordinates_finite=result.all_coordinates_finite&&finite(p);
        }
        return same;
    };
    geometry.DeflectControlSurfaces();
    result.deflect_wake_matches_fine=check_wake();
    result.deflect_fine_nodes_unchanged=fine_unchanged();
    geometry.UnDeflectControlSurfaces();
    result.undeflect_wake_matches_fine=check_wake();
    result.undeflect_fine_nodes_unchanged=fine_unchanged();
    return result;
}

static void json_result(std::ostream& out,const Result& r) {
    out<<"{\"name\":\""<<r.name<<"\",\"level\":"<<r.level<<",\"available_levels\":"<<r.levels
       <<",\"input_vertices\":"<<r.input_vertices<<",\"reported_surface_count\":"<<r.reported_surface_count
       <<",\"actual_surface_nodes\":"<<r.actual_surface_nodes<<",\"unused_vertex\":"<<r.unused<<",\"nonidentity\":"<<r.nonidentity
       <<",\"surfaces_unchanged\":"<<r.surfaces_unchanged<<",\"other_fine_nodes_unchanged\":"<<r.other_fine_nodes_unchanged
       <<",\"roots_attached\":"<<r.roots_attached<<",\"repeated_update_idempotent\":"<<r.repeated_update_idempotent
       <<",\"all_coordinates_finite\":"<<r.all_coordinates_finite
       <<",\"deflect_wake_matches_fine\":"<<r.deflect_wake_matches_fine
       <<",\"undeflect_wake_matches_fine\":"<<r.undeflect_wake_matches_fine
       <<",\"deflect_fine_nodes_unchanged\":"<<r.deflect_fine_nodes_unchanged
       <<",\"undeflect_fine_nodes_unchanged\":"<<r.undeflect_fine_nodes_unchanged
       <<",\"zero_control_surfaces\":0,\"checked_wake_nodes_per_path\":"<<r.checked_wake_nodes
       <<",\"passed\":"<<r.passed()<<",\"attachments\":[";
    for(std::size_t i=0;i<r.attachment.size();++i) {
        if(i)out<<',';const auto& a=r.attachment[i];
        out<<"{\"te\":"<<a.te<<",\"wake\":"<<a.wake<<",\"fine_te\":"<<a.fine_te<<",\"fine_wake\":"<<a.fine_wake<<'}';
    }
    out<<"]}";
}

int main(int argc,char** argv) {
    try {
        require(argc==2,"usage: vsp_wake_attachment NEW_OUTPUT_DIRECTORY");
        const auto output=std::filesystem::absolute(argv[1]);
        require(!std::filesystem::exists(output),"output directory already exists");
        std::filesystem::create_directories(output);
        std::vector<Result> results;
        results.push_back(run_case(geometries[0],output,"level0-identity",0,false));
        results.push_back(run_case(geometries[1],output,"level1-identity",1,false));
        results.push_back(run_case(geometries[2],output,"level1-unused-nonidentity",1,true));
        results.push_back(run_case(geometries[3],output,"level2-unused-nonidentity",2,true));
        bool all=true;for(const auto& r:results)all=all&&r.passed();
        std::ofstream report(output/"report.json");report<<std::boolalpha;
        report<<"{\"scope\":\"Public-library geometry attachment indexing regression; no flow\",\"all_passed\":"<<all
              <<",\"model_accepted\":false,\"cases\":[";
        for(std::size_t i=0;i<results.size();++i){if(i)report<<',';json_result(report,results[i]);}
        report<<"]}\n";report.close();require(bool(report),"report write failed");
        for(const auto& r:results)std::printf("FSIM_ATTACHMENT_CASE %s passed=%d nonidentity=%d surface_immutable=%d roots_attached=%d\n",r.name,int(r.passed()),int(r.nonidentity),int(r.surfaces_unchanged),int(r.roots_attached));
        return all?0:2;
    } catch(const std::exception& error) {
        std::fprintf(stderr,"FSIM_ATTACHMENT_FIXTURE_INVALID %s\n",error.what());return 3;
    }
}
