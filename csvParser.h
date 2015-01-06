#include <vector>
#include <string>
#include <stdlib.h>

#include <boost/tuple/tuple.hpp>

#include <barrett/products/product_manager.h>
//#include <barrett/systems.h>
#include <barrett/units.h>

#include "csv_parser.hpp"

using namespace barrett;

template<typename T, size_t DOF>
std::vector<T> csvParser(char* filename)
{
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	std::vector<T> vec;

	T buff;
	double buff1;
	jp_type buff2;

	const char * fn = filename;
	//const char * filename = "traj";
	const char field_terminator = ',';
	const char line_terminator  = '\n';
	const char enclosure_char   = '"';

	csv_parser file_parser;
	/* Define how many records we're gonna skip. This could be used to skip the column definitions. */
	//file_parser.set_skip_lines(0);

	/* Specify the file to parse */
	file_parser.init(fn);

	/* Here we tell the parser how to parse the file */
	file_parser.set_enclosed_char(enclosure_char, ENCLOSURE_OPTIONAL);

	file_parser.set_field_term_char(field_terminator);

	file_parser.set_line_term_char(line_terminator);

	unsigned int row_count = 1U;
	unsigned int i = 0;

	/* Check to see if there are more records, then grab each row one at a time */
	while(file_parser.has_more_rows())
	{
		i = 0;
	    /* Get the record */
	    csv_row row = file_parser.get_row();

	    /* Print out each column in the row */
		for (i = 0; i < row.size(); i++)
		{
		   	//printf("COLUMN %02d : %s\n", i + 1U, row[i].c_str());
		    if (i == 0) buff1 = atof(row[i].c_str());
		    else buff2[i-1] = atof(row[i].c_str());
		}

		boost::get<0>(buff) = buff1;
		boost::get<1>(buff) = buff2;

		vec.push_back(buff);

		/*std::cout<< boost::get<0>(buff) << boost::get<1>(buff)<<std::endl;
		printf("END OF ROW %02d\n", row_count);
		printf("====================================================================\n");
*/
		row_count++;
	}

	return vec;
}
