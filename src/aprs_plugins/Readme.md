
# C++ Plugins
----



This readme gives a brief overview of the plugin strategy used in gzrcs. 


C++ plugins were based on the boost DLL package. Unfortunately, boost DLL requires version 1.61 of boost, and Ubuntu 16.04 only supports 1.54. Unfortunately, I did not have sufficient sudo privileges to upgrade boost to version 1.61  Fortunately, Boost.DLL is a header only library and is available on github at https://github.com/boostorg/dll. So only boost DLL was downloaded and ran in place with boost version 1.54. Thankfully no surprises were encountered. As a header only boost package, the include boost DLL header files were put under the gzaprs-xenial include directory and referenced by those ROS packages that used the plugin tools. Of note, the boost DLL package does require the "dl" library to dynamically load/unload shared object libraries (.so files).
## <a name="Factory_method_in_plugin"></a>Factory method in plugin



Kinematic solver plugins were first chosen, and used the factory strategy shown in the boost DLL documentation. From boost DLL documentation found https://www.boost.org/doc/libs/1_61_0/doc/html/boost_dll/tutorial.html#boost_dll.tutorial.factory_method_in_plugin one finds a factory generator for plugins that was used. The following extracts from that tutorial to describe the kinematic solver pluings. First, an GoKin kinematic solver instance of the abstract C++ class IKInematic is built as a shared library project in both ROS catkin_make and QT qmake. The header file is shown below, where IKinematic is a header defined in the aprs_header folder and GoKin derives from IKinematic and then defines a "create" method for creation of GoKin instances. 
	#include <boost/dll/alias.hpp> // for BOOST_DLL_ALIAS   
	#include <aprs_headers/IKinematic.h>
	namespace RCS {
	class GoKin: public IKinematic {
	public:
	    float calculate(float x, float y) {
	        aggr_ += x + y;
	        return aggr_;
	    }
	    // Factory method
	    static boost::shared_ptr<IKinematic> create() {
	        return boost::shared_ptr< IKinematic >(
	            new GoKin ()
	        );
	    }
	};
	BOOST_DLL_ALIAS(
	    RCS::GoKin::create, // <-- this function is exported with...
	    create_plugin       // <-- ...this alias name
	)
	} // namespace RCS


The meat of IKinematic methods (FK & IK) are omitted here. Basically, the problem is that each C++ compiler mangles symbols in a different manner, but C does not, so a C style declaration is used to allow finding the create symbol in the dll/so.


As you may see, RCS::create  is a factory method, that creates instances of RCS::GoKin. We export that method with the name "create_plugin" using BOOST_DLL_ALIAS.


The function that load the shared DLL library and then creates the GoKin instance is outline below.
	#include <boost/dll/import.hpp> // for import_alias
	#include <boost/function.hpp>
	#include <iostream>
	#include <aprs_headers/IKinematic.h>
	namespace dll = boost::dll;
	int main(int argc, char* argv[]) {
	    // argv[1] contains path to directory with our plugin library
	    boost::filesystem::path shared_library_path(argv[1]);
	    shared_library_path /= "gokin_plugin";  // name of gokin solver project dll
	    typedef boost::shared_ptr<my_plugin_api> (pluginapi_create_t)();
	    boost::function<pluginapi_create_t> creator;
	    // type of imported symbol must be explicitly specified  
	    creator = boost::dll::import_alias<pluginapi_create_t>(   
	        shared_library_path,                        // path to library
	        "create_plugin",                            //symbol to import
	        Boost::dll::load_mode::append_decorations  // append extensions and prefixes
	    );
	    boost::shared_ptr<my_plugin_api> plugin = creator();
	    std::cout << "plugin->calculate(1.5, 1.5) call:  " << plugin->calculate(1.5, 1.5) << std::endl;
	    std::cout << "plugin->calculate(1.5, 1.5) second call:  " << plugin->calculate(1.5, 1.5) << std::endl;
	    std::cout << "Plugin Name:  " << plugin->name() << std::endl;
	}


Note, DO NOT LET boost::shared_ptr<my_plugin_api> plugin go out of scope and be destroyed. If you do, you will get a segmentation fault, as the shared pointer will not be NULL but will point to garbage.








