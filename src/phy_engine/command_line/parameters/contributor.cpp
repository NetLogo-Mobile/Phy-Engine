#include "../../../phy_engine_utils/fast_io/fast_io.h"

#include "contributor.h"
#include "../../devices/native_io.h"
#include "../../contributor/contributor.h"
#include "../../../phy_engine_utils/ansies/impl.h"

::phy_engine::command_line::parameter_return_type(::phy_engine::parameter::details::contributor_callback)(::std::size_t, ::fast_io::vector<::phy_engine::command_line::parameter_parsing_results>&) noexcept {
	::fast_io::basic_obuf<::fast_io::u8native_io_observer> buf_u8out{::phy_engine::u8out};
	::fast_io::print(buf_u8out,
					 ::phy_engine::ansi_escape_sequences::rst::all,
					 ::phy_engine::ansi_escape_sequences::col::white,
					 u8"CONTRIBUTOR:\n",
					 ::phy_engine::ansi_escape_sequences::rst::all);
	for (::std::size_t i{}; i < ::phy_engine::contributor_size; i++) 
		::fast_io::println(buf_u8out, u8"\t", ::phy_engine::contributor[i]);
	return ::phy_engine::command_line::parameter_return_type::def;
}