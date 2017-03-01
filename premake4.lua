
solution "famous"
   configurations { "debug", "release", "profile" }
 
   project "famous"
      kind "StaticLib"
      language "C++"
      files { "src/**.h", "src/**.cpp" }
 
      includedirs { "src/", "src/objects", "src/services", "src/devices"}
 
      if os.is ("linux") then
     	    includedirs { "/usr/include/bullet" }

      elseif os.is ("macosx") then
     	     includedirs { "/opt/local/include/bullet" }
             includedirs { "/opt/local/include" }
      end


      configuration "release"
         buildoptions {"-std=c++11"}
         defines { "NDEBUG" }
         flags { "OptimizeSpeed", "EnableSSE", "EnableSSE2", "FloatFast", "NoFramePointer"}    

      configuration "debug"
         buildoptions {"-std=c++11"}
         defines { "DEBUG" }
         flags { "Symbols" }

      configuration "profile"
         buildoptions {"-std=c++11"}
         defines { "PROFILE" }
         flags { "Symbols" }
	 buildoptions { "-pg" }	


if _ACTION == "clean" then
  os.execute ("rm -f *.deb")
end


newaction {
   trigger     = "install",
   description = "Install the software",
   execute     = function ()
      if os.is ("linux") then
          os.execute ("mkdir -p /usr/local/include/libfamous/")
          os.execute ("cp libfamous.a /usr/local/lib/")
     	  os.execute ("find ./src/ -name *.h -exec cp -u {} /usr/local/include/libfamous/ \\;")
     	  os.execute ("echo Files have been copied to /usr/local/lib and /usr/local/include/libfamous")
      elseif os.is ("macosx") then
          os.execute ("mkdir -p /opt/local/include/libfamous/")
          os.execute ("cp libfamous.a /opt/local/lib/")
     	  os.execute ("find ./src/ -name *.h -exec cp {} /opt/local/include/libfamous/ \\;")
     	  os.execute ("echo Files have been copied to /opt/local/lib and /opt/local/include/libfamous")
      end
   end
}

newaction {
   trigger     = "debpackage",
   description = "Package the software in a .deb file",
   execute     = function ()
      if os.is ("linux") then
          os.execute ("mkdir libfamous-3.0")
          os.execute ("cp -a debian libfamous-3.0/DEBIAN")
          os.execute ("mkdir -p libfamous-3.0/usr/include/libfamous")
          os.execute ("mkdir -p libfamous-3.0/usr/lib")
          os.execute ("cp libfamous.a libfamous-3.0/usr/lib/")
     	  os.execute ("find ./src/ -name *.h -exec cp -u {} libfamous-3.0/usr/include/libfamous/ \\;")
          os.execute ("dpkg-deb --build libfamous-3.0")
	  os.execute ("rm -rf libfamous-3.0")
     	  os.execute ("echo Debian package created")
      elseif os.is ("macosx") then
     	  os.execute ("echo No package on Mac OSX, please use premake4 install")
      end
   end
}
