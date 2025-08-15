#include "StructComponents.hpp"
std::string StatusToString(Status givenStatus){
	switch(givenStatus){
	    case Danger:
		    return "DANGER";
	    case Warning:
		    return "Warning";
	    case Success:
		    return "Success";
	    case Standby:
		    return "Standby";
        default:
            exit(42);
	}
    
}