package frckit.codegen;

import com.squareup.javapoet.CodeBlock;
import com.squareup.javapoet.JavaFile;
import com.squareup.javapoet.MethodSpec;
import com.squareup.javapoet.TypeSpec;

import javax.annotation.processing.*;
import javax.lang.model.SourceVersion;
import javax.lang.model.element.*;
import javax.lang.model.type.TypeMirror;
import javax.tools.Diagnostic;
import java.io.IOException;
import java.util.Set;

@SupportedAnnotationTypes("frckit.codegen.GenerateDefault")
@SupportedSourceVersion(SourceVersion.RELEASE_11)
public class GenerateDefaultAnnotationProcessor extends AbstractProcessor {

    @Override
    public boolean process(Set<? extends TypeElement> annotations, RoundEnvironment roundEnv) {
        for (Element element : roundEnv.getElementsAnnotatedWith(GenerateDefault.class)) {
            if (element.getKind().isInterface() || (element.getKind().isClass() && element.getModifiers().contains(Modifier.ABSTRACT))) {
                TypeElement typeElement = (TypeElement) element; //Cast the root element to a TypeElement

                //Create a builder which is used to generate the Default implementation class
                TypeSpec.Builder typeBuilder = TypeSpec.classBuilder(typeElement.getSimpleName() + "Default")
                        .addModifiers(Modifier.PUBLIC, Modifier.FINAL); //TODO match visibility modifier of the superclass

                //Determine if the superclass is an interface or an abstract class (decides if we use 'implements' or 'extends')
                if (element.getKind().isInterface()) {
                    typeBuilder.addSuperinterface(typeElement.asType());
                } else {
                    typeBuilder.superclass(typeElement.asType());
                }

                //Iterate over each inner element of the class (this includes fields, methods, and subclasses)
                for (Element enclosedElement : typeElement.getEnclosedElements()) {
                    //Filter elements that are only abstract methods
                    if (enclosedElement.getKind() == ElementKind.METHOD && enclosedElement.getModifiers().contains(Modifier.ABSTRACT)) {
                        ExecutableElement executableElement = (ExecutableElement) enclosedElement;
                        MethodSpec.Builder methodBuilder = MethodSpec.overriding(executableElement);

                        //Check if the method is annotated with a "DefaultReturn"
                        DefaultReturn defaultReturn = executableElement.getAnnotation(DefaultReturn.class);
                        if (defaultReturn != null) {
                            //A default return value was provided, use it
                            methodBuilder.addCode("return " + defaultReturn.value() + ";");
                        } else {
                            //No value provided, use a predetermined default.
                            switch (executableElement.getReturnType().getKind()) {
                                case BOOLEAN:
                                    methodBuilder.addCode("return false;");
                                    break;
                                case BYTE:
                                case SHORT:
                                case INT:
                                case LONG:
                                case CHAR:
                                case FLOAT:
                                case DOUBLE:
                                    methodBuilder.addCode("return 0;");
                                    break;
                                default:
                                    methodBuilder.addCode("return null;");
                                    break;
                            }
                        }

                        //Add the generated method to the type builder
                        typeBuilder.addMethod(methodBuilder.build());
                    }
                }

                try {
                    JavaFile.builder(processingEnv.getElementUtils().getPackageOf(typeElement).getQualifiedName().toString(), typeBuilder.build()).build().writeTo(processingEnv.getFiler());
                } catch (IOException e) {
                    e.printStackTrace();
                }
            } else {
                processingEnv.getMessager().printMessage(Diagnostic.Kind.ERROR, "Only interfaces or abstract classes can be annotated with @" + GenerateDefault.class.getSimpleName(), element);
                return true;
            }
        }
        return false;
    }
}
