set_languages("c++20")

for index, file in ipairs(os.files("**.cpp")) do
    local name = path.basename(file)
    local dir = path.directory(file)
    target(name) do
        before_build(function (target)
            os.mkdir("build/test/" .. dir)
        end)
        set_kind("binary")
        set_targetdir("build/test/" .. dir)
        set_default(false)
        add_files(file)
        add_tests("default")
        add_includedirs("../include")
    end
end
