# Setting up Minimal Chisel Development Environment with Mill

The project is [here](https://github.com/ucb-bar/A-Minimal-Chisel-Project/tree/main), with instructions of setting everything up in the README file.&#x20;



So here, instead of reiterrating the set up flow, we will document some design choices of doing things.



### Scala build file

Mill will look for build files named as \`build.sc\`, \`build.mill\`, or \`build.mill.scala\`. We decided to use `build.mill.scala`as it makes it cleaner that a) this is a build file for mill, and b) this is a file with scala syntax. `build.mill`is another decent option, but unfortunately Github does not recognize this file extension name, and cannot properly do code highlighting.



### Source directory organization

For some reason, the build system cannot find the Elaborate.scala file if it is placed under `/src/main/scala/`. Instead, the current build system searches for source files under `<TopLevelObjectName>/src/main` and `<TopLevelObjectName>/src/main/scala`.

Therefore, we now have this a bit awkward `YourChiselProject` folder.



